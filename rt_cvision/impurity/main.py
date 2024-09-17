
import os
import cv2
import uuid
import logging
import threading
import numpy as np
from typing import Optional, Dict
from impurity.tasks.annotate.base import draw
from impurity.tasks.email.notifify import send_email
from common_utils.detection.utils import box_iou_batch
from impurity.tasks.video.generate import generate_video
from common_utils.detection.core import Detections
from impurity.tasks.database.register import save_results_into_db
from impurity.tasks.snapshot.save import save_snapshot, save_experiment
from impurity.tasks.check_objects import (
    check_object_severity_level,
    check_object_size,
    check_object_problematic,
)

from configure.client import config_manager
parameters = config_manager.params.get('impurity')

tasks:dict = {
    'draw': draw,
    'save_snapshot': save_snapshot,
    'save_experiment': save_experiment,
    'generate_video': generate_video,
    'send_email': send_email,
    'save_results_into_db': save_results_into_db, 
}

mapping_threshold:list = [0., 0.5, 1.]
mapping_colors:list = [(0, 255, 0), (0, 255, 255), (0, 0, 255)],
mapping_key:str = 'object_length'
iou_threshold:float = parameters.get('iou_threshold')
line_width:int = 3

snapshot_dir:str = parameters.get('snapshot_dir')
experiment_dir:str =  parameters.get('experiment_dir')
db_url:str = parameters.get('db_url')
email_url:str = parameters.get('email_url')
video_url:str = parameters.get("video_url")
classes:list = [1, 2]


class Processor:
    def __init__(self) -> None:
        self.map_tracker_id_2_object_uid = {}
    
    def execute(self, detections:Detections, cv_image:np.ndarray, data:Optional[Dict]=None, classes=None):
        try:
            if classes:
                detections = detections[np.isin(detections.class_id, classes)]
            
            if not len(detections):
                print('No Detection ! 🕵️‍♂️🔍❌ ')
                return 
            
            objects_seg = check_object_size.check(
                objects=data,
                threshold=[0., 0.5, 1.],
            )    
        
            if not objects_seg.get('xyxyn'):
                return
            
            print(f'Detecting {len(detections)} potential impurity vs {len(objects_seg.get("xyxyn"))} Segments ! Analyis started 🔍 ... ...')
            problematic_objects = check_object_problematic.is_object_problematic(
                detections=detections, segments=objects_seg, iou_threshold=iou_threshold, mapping_key=mapping_key, mapping_threshold=mapping_threshold,
            )
            
            problematic_objects = self.register_objects(problematic_objects)
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
                }
            )
            
            params = {
                "cv_image": cv_image,
                "snapshot": alarm_image,
                "snapshot_dir": snapshot_dir,
                "filename": data.get('filename'),
                "experiment_dir": experiment_dir,
                "timestamp": data.get('datetime'),
                "severity_level": problematic_objects.get('severity_level'),
                "event_uid": str(uuid.uuid4()),
                "event_description": f"{len(problematic_objects.get('severity_level', []))} prob. Langteile: {problematic_objects.get('object_length')}",
                "snapshot_url": f"/alarms/snapshots/stoerstoff/{data.get('filename')}",
                "snapshot_id": str(uuid.uuid4()),
                "model_name": parameters.get('weights'),
                "model_tag": 'v003',
                "db_url": db_url,
                "objects": problematic_objects,
                "delivery_id": "keine Zuordnung",
                "meta_info": {
                    "description": f"{len(problematic_objects.get('severity_level', []))} prob. Langteile: {problematic_objects.get('object_length')}",
                }
            }
            
            for key, value in parameters.get('tasks', {}).items():
                func = tasks.get(key)
                if value:
                    print(f"Executing {key} ... ", end='')
                    func(params)
                    print("Done")
            
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
        assert 'xyn' in objects.keys(), f"key: xyn not found in objects"
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


if __name__ == "__main__":
    from common_utils.detection.utils import box_iou_batch
    