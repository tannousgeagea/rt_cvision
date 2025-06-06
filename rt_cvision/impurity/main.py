
import os
import gc
import cv2
import uuid
import time
import torch
import logging
import threading
import numpy as np
from typing import Optional, Dict
from datetime import datetime, timezone
from impurity.tasks.annotate.base import draw
from impurity.tasks.email.notifify import send_email
from common_utils.detection.utils import box_iou_batch
from impurity.tasks.video.generate import generate_video
from common_utils.detection.core import Detections
from impurity.tasks.delivery.core  import get
from impurity.tasks.database.register import save_results_into_db
from impurity.tasks.edge_to_cloud.core import sync_media_to_cloud
from impurity.tasks.snapshot.save import save_snapshot, save_experiment
from impurity.tasks import plant_controller
from common_utils.material_cls.core import filter_from_material
from common_utils.duplicate_tracker.core import DuplicateTracker
from common_utils.timezone_utils.timeloc import get_location_and_timezone, convert_to_local_time
from common_utils.severity_refiner.core import SeverityLevelXDetector
from impurity.tasks.check_objects import (
    check_object_severity_level,
    check_object_size,
    check_object_problematic,
)


from common_utils.roi.utils import _get_center, _is_within
from configure.client import config_manager, entity, sensorbox, entity_loc
parameters = config_manager.params.get('impurity')
timezone_str = get_location_and_timezone()
DATETIME_FORMAT = "%Y-%m-%d %H:%M:%S"

tasks:dict = {
    'draw': draw,
    'save_snapshot': save_snapshot,
    'save_experiment': save_experiment,
    'generate_video': generate_video,
    'send_email': send_email,
    'save_results_into_db': save_results_into_db,
    "sync_media_to_cloud": sync_media_to_cloud,
    "notify_plant_controller": plant_controller.core.execute,
}

mapping_threshold:list = [0., 0.3, 0.5, 1.]
mapping_colors:list = [(0, 255, 0), (0, 255, 255), (0, 165, 255), (0, 0, 255)]
mapping_key:str = parameters.get("severity_mapping_key", 'object_length')

if "object_length_threshold" in parameters.keys():
    mapping_threshold = parameters.get('object_length_threshold')

if "mapping_object_color" in parameters.keys():
    mapping_colors = parameters.get("mapping_object_color")
    
iou_threshold:float = parameters.get('iou_threshold')
line_width:int = 3

snapshot_dir:str = parameters.get('snapshot_dir')
experiment_dir:str =  parameters.get('experiment_dir')
db_url:str = parameters.get('db_url')
email_url:str = parameters.get('email_url')
video_url:str = parameters.get("video_url")
topic:str = parameters.get('topic', "/front/rgb_left")
edge_2_cloud_url:str = parameters.get('edge_2_cloud_url')
delivery_api_url:str = parameters.get('delivery_api_url')
share_edgebox_id:bool = parameters.get('share_edgebox_id', False)
edge_2_cloud_url_data = parameters.get('edge_2_cloud_url_data')
pc_url = parameters.get("pc_url")
classes:list = [1, 2]


class Processor:
    def __init__(self) -> None:
        self.map_tracker_id_2_object_uid = {}
        self.tracker = DuplicateTracker(buffer_size=10, iou_threshold=0.5, expiry_minutes=10)
        self.roi = parameters.get("roi")

        self.severity_refiner = None
        severity_level_detector = parameters.get('severity_level_detector', {})
        if severity_level_detector and severity_level_detector.get('is_active', False):
            self.severity_refiner = SeverityLevelXDetector(
                model_path=severity_level_detector.get('model_path'), 
                mlflow=severity_level_detector.get('mlflow'),
                conf_threshold=severity_level_detector.get('conf_threshold', 0.25),
                X=severity_level_detector.get('X', 2),
                cf=config_manager.params.get('segmentation').get('correction_factor'),
                filter_config=parameters.get("filter_config"),
                )
    
    def execute(self, detections:Detections, cv_image:np.ndarray, data:Optional[Dict]=None, classes=None):
        try:
            start_time = time.time()
            if classes:
                detections = detections[np.isin(detections.class_id, classes)]
            
            if not len(detections):
                print('No Detection ! 🕵️‍♂️🔍❌ ')
                return 
            
            objects_seg = check_object_size.check(
                objects=data,
                threshold=mapping_threshold,
            )    
        
            if not objects_seg.get('xyxyn'):
                return
            
            print(f'Detecting {len(detections)} potential impurity vs {len(objects_seg.get("xyxyn"))} Segments ! Analyis started 🔍 ... ...')
            problematic_objects = check_object_problematic.is_object_problematic(
                detections=detections, 
                segments=objects_seg, 
                iou_threshold=iou_threshold, 
                mapping_key=mapping_key, 
                mapping_threshold=mapping_threshold,
            )

            problematic_objects = self.register_objects(problematic_objects)
            problematic_objects = filter_from_material(problematic_objects, cv_image, filter_materials=['PLASTIC'])
            
            if self.severity_refiner and self.severity_refiner.model:
                problematic_objects = self.severity_refiner.integrate(problematic_objects, cv_image, iou_threshold=0.5)

            if not problematic_objects.get('xyxyn'):
                return

            problematic_objects = self.check_duplicate(objects=problematic_objects)

            if not problematic_objects.get('xyxyn'):
                return
            
            labels = [
                f'{int(mapping_threshold[i] * 100)} - {int(mapping_threshold[i+1] * 100)} cm'
                for i in range(len(mapping_threshold) - 1)
            ] + [f'> {int(mapping_threshold[-1] * 100)} cm']

            alarm_image = draw(
                params={
                    "cv_image": cv_image.copy(),
                    "line_width": line_width,
                    "colors": mapping_colors,
                    "objects": problematic_objects,
                    "legend": labels,
                    "entity_loc": entity_loc,
                }
            )
            
            within_roi = True

            logging.info(f"ROI: {self.roi}")
            if self.roi:
                h, w, _ = cv_image.shape
                x_coords, y_coords = zip(*self.roi)
                x_min, x_max = min(x_coords), max(x_coords)
                y_min, y_max = min(y_coords), max(y_coords)
                roi = [x_min, y_min, x_max, y_max]
                centers = [_get_center(bbox) for bbox in problematic_objects.get('xyxyn', [])]
                within_roi = [_is_within(point=center, roi=roi) for center in centers]
                within_roi = any(within_roi)

            if within_roi:
                delivery_id = get(
                    url=delivery_api_url,
                    params={
                        "timestamp": datetime.now(tz=timezone.utc)
                    }
                )
            else:
                logging.info("All objects are outside ROI")
                delivery_id = None
            
            event_uid = str(uuid.uuid4())
            filename = (
                f"{entity.entity_type.tenant.tenant_name}_"
                f"{entity.entity_uid}_"
                f"{sensorbox.sensor_box_location}_"
                f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_"
                f"{event_uid}.jpg"
            )
            
            params = {
                "cv_image": cv_image,
                "snapshot": alarm_image,
                "snapshot_dir": snapshot_dir,
                "filename": filename,
                "experiment_dir": experiment_dir,
                "timestamp": convert_to_local_time(datetime.now(), timezone_str=timezone_str).strftime(DATETIME_FORMAT),
                "severity_level": problematic_objects.get('severity_level'),
                "event_uid": event_uid,
                "event_description": f"{len(problematic_objects.get('severity_level', []))} prob. Langteile: {problematic_objects.get('object_length')}",
                "snapshot_url": f"{str(entity.entity_type.tenant.tenant_name).lower()}/alarm/impurity/{filename}",
                "snapshot_id": str(uuid.uuid4()),
                "model_name": parameters.get('weights'),
                "model_tag": 'v003',
                "db_url": db_url,
                "video_url": video_url,
                "email_url": email_url,
                "pc_url": pc_url,
                "gate_id": os.getenv('GATE_ID', 'gate03'),
                "topic": topic,
                "objects": problematic_objects,
                "delivery_id": delivery_id,
                "edge_2_cloud_url": edge_2_cloud_url,
                "edge_2_cloud_url_data": edge_2_cloud_url_data,
                "media_file": f"{snapshot_dir}/{filename}",
                "location": f"{entity.entity_uid}",
                "location_loc": f"{entity_loc}",
                "tenant_domain": f"{entity.entity_type.tenant.domain}",
                "tenant_location": f"{entity.entity_type.tenant.location}",
                "meta_info": {
                    "description": f"{len(problematic_objects.get('severity_level', []))} prob. Langteile: {problematic_objects.get('object_length')}",
                    "object_length": problematic_objects.get('object_length'),
                    "object_area": problematic_objects.get("object_area"),
                    "value": max(problematic_objects.get('object_length')),
                    "unit": "cm",
                }
            }
            
            for key, value in parameters.get('tasks', {}).items():
                func = tasks.get(key)
                if value:
                    print(f"Executing {key} ... ", end='')
                    func(params)
                    print("Done")

            
            del cv_image
            del objects_seg
            del problematic_objects

            gc.collect()
            torch.cuda.empty_cache()
            max_memory_usage = torch.cuda.max_memory_allocated() / (1024 * 1024)
            reserved_memory = torch.cuda.max_memory_reserved() / (1024 * 1024)

            print(f'Memory Usage: {max_memory_usage} mb')
            print(f'Reserved Memory Usage: {reserved_memory} mb')
            print('Total Execution Time:::::> %s s' % round(time.time() - start_time, 4))
            
        except Exception as err:
            logging.error(f"Error while executing detections in impurity: {err}")
            
    
    def register_objects(self, objects:dict):
        """
        Registers objects by assigning unique IDs (object_uid) and updating internal mapping
        of tracker_id to object_uid. If tracker_id is already known, it reuses the existing object_uid.

        Args:
            objects (dict): Dictionary containing object data with keys such as 'tracker_id' and 'xyn'.

        Returns:
            new_objects (dict): Dictionary of newly registered objects.
            objects (dict): Original dictionary with updated 'object_uid' for known tracker_ids.
        """
        assert 'xyxyn' in objects.keys(), f"key: xyxyn not found in objects"
        object_tracker_id = objects.get('tracker_id', [])
                
        if not object_tracker_id:
            return objects
             
        new_objects = objects.copy()   
        try:
            new_objects = {
                key:[
                    value[i] for i in range(len(value))
                    if object_tracker_id[i] not in self.map_tracker_id_2_object_uid
                ] 
                for key, value in new_objects.items()
                if isinstance(value, list)
            }
            
            for i, tracker_id in enumerate(object_tracker_id):
                if tracker_id in self.map_tracker_id_2_object_uid.keys():
                    continue
                
                self.map_tracker_id_2_object_uid[tracker_id] = objects.get('object_uid')[i]
                       
        except Exception as err:
            logging.error(f'Unexpected Error while registering objects: {err}')
            
        return new_objects
    
    def check_duplicate(self, objects:dict):
        try:
            unique_indices = []
            for i, xyxy in enumerate(objects['xyxyn']):
                temp_det = {"bbox": xyxy, "timestamp": time.time()}
                if self.tracker.add_detection(temp_det):
                    logging.info(f"Detection {i} is unique and added to the buffer.")
                    unique_indices.append(i)
                else:
                    logging.info(f"Detection {i} is a duplicate.")
            
            logging.info(objects)
            logging.info(unique_indices)
            
            objects = {
                key: [value[i] for i in unique_indices]
                for key, value in objects.items()
                if isinstance(value, list)
            }
        except Exception as err:
            logging.error(f'Unexpected Error while checking duplicate: {err}')
        
        return objects