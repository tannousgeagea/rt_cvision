
import os
import cv2
import uuid
import logging
import threading
import numpy as np
from typing import Optional, Dict
from datetime import datetime, timezone
from common_utils.model.base import BaseModels
from impurity.utils.objects.core import ObjectManager
from common_utils.detection.core import Detections

from configure.client import ConfigManager
config_manager = ConfigManager()

model = BaseModels(
    weights=config_manager.impurity.weights, 
    task='impurity', 
    mlflow=config_manager.impurity.mlflow.get('active', False)
)

mapping_threshold:list = [0., 0.3, 0.5, 1.]
mapping_colors:list = [(0, 255, 0), (0, 255, 255), (0, 165, 255), (0, 0, 255)]
mapping_key:str = 'object_length'
classes:list = [1, 2]

class Processor:
    def __init__(self) -> None:
        self.map_tracker_id_2_object_uid = {}
    
    def execute(self, cv_image:np.ndarray, data:Optional[Dict]=None, classes=None):
        try:
            data = self.register_objects(data)
            segments = Detections.from_dict(data)

            print(segments.object_length)
            indices = np.where(segments.object_length >= mapping_threshold[1])[0]
            segments = segments[indices]
            
            if not len(segments):
                print('No Qualified Objects')
                return
            
            detections = model.classify_one(cv_image, conf=config_manager.impurity.conf, is_json=False)
            if classes:
                detections = detections[np.isin(detections.class_id, classes)]
            
            if not len(detections):
                print('No Detection ! 🕵️‍♂️🔍❌ ')
                return 
            
            detections = detections.with_nms()
            object_manager = ObjectManager(
                segments=segments, detections=detections
            )
            
            detections = object_manager.is_problematic()
            if detections is None:
                return

            object_manager.severtiy_level(mapping_key=mapping_key, mapping_threshold=mapping_threshold)
            object_manager.save(cv_image=cv_image)
            return detections
            
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
        assert 'xyxyn' in objects.keys(), f"key: xyn not found in objects"
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