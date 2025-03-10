import os
import time
import uuid
import logging
import random
import numpy as np
from segments.tasks.database import register
from common_utils.detection.core import Detections
from segments.tasks.object_size.estimate_object_size import ObjectSizeEst
from segments.tasks.publish.core import publish_to_kafka
from common_utils.timezone_utils.timeloc import get_location_and_timezone, convert_to_local_time
from configure.client import ConfigManager

timezone_str = get_location_and_timezone()
logging.info(f"Local Timezone: {timezone_str}")
config_manager = ConfigManager()

tasks = {
    'publish_to_kafka': publish_to_kafka,
    'save_results_into_db': register.save_results_into_db, 
}

class Processor:
    def __init__(self) -> None:
        self.map_tracker_id_2_object_uid = {}
        self.object_size_est = ObjectSizeEst()
    
    def execute(self, cv_image, detections:Detections, data:dict):
        try:
            objects = detections.to_dict()

            start_est_size = time.time()
            objects = self.object_size_est.execute(
                objects=objects, 
                input_shape=cv_image.shape,
                correction_factor=config_manager.segmentation.correction_factor,
                )
            print(f"5. Estimatint Size: {round((time.time() - start_est_size) * 1000, 2)} ms")

            start_tracking = time.time()
            instances, all_instances = self.register_objects(objects=objects)
            print(f"6. Estimatint Size: {round((time.time() - start_tracking) * 1000, 2)} ms")

            h0, w0, _ = cv_image.shape

            kafka_msg = {
                **data,
                **all_instances,
            }
            
            params = {
                'message': kafka_msg,
                'db_url': config_manager.segmentation.db_url,
                'objects': instances,
                'model_name': 'wasteant-segmentation',
                'model_tag': 'v003',
                'meta_info': {
                    'height': h0,
                    'width': w0,
                }
            }
            
            for key, value in config_manager.segmentation.tasks.items():
                start_task = time.time()
                func = tasks.get(key)
                if value:
                    print(f'Executing {key} ... ', end='')
                    func(params)
                    print('Done !')
                print(f"7. Task {key}: {round((time.time() - start_task) * 1000, 2)} ms")

        except Exception as err:
            logging.error(f"Error while executing detections in segments: {err}")
            
        
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
        objects['object_uid'] = [str(uuid.uuid4()) for _ in range(len(objects['xyxyn']))]
                
        if not object_tracker_id:
            objects['tracker_id'] = [random.randint(100000, 999999) for _ in objects['object_uid']]
            return objects, objects
             
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
                    objects['object_uid'][i] = self.map_tracker_id_2_object_uid[tracker_id]
                    continue
                
                self.map_tracker_id_2_object_uid[tracker_id] = objects.get('object_uid')[i]
                       
        except Exception as err:
            logging.error(f'Unexpected Error while registering objects: {err}')
            
        return new_objects, objects
