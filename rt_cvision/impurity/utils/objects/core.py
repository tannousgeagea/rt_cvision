import cv2
import uuid
import logging
import numpy as np
from typing import Union
from typing import Optional, Dict
from common_utils.detection.utils import box_iou_batch
from common_utils.detection.core import Detections
from .severity_level_map import SEVERITY_MAP
from datetime import datetime, timezone
import django
django.setup()
from django.core.files.base import ContentFile
from data_reader.models import (
    Image
)

from impurity.models import (
    Impurity,
)

from configure.client import (
    sensorbox
)

class ObjectManager():
    def __init__(self, segments:Union[Dict, Detections], detections:Detections):
        self.segments = segments if isinstance(segments, Detections) else Detections.from_dict(segments) 
        self.detections = detections
    
    def is_problematic(
        self,
        iou_threshold:float=0.45,
        ):
        
        try:
            ious = box_iou_batch(
                self.segments.xyxyn,
                self.detections.xyxyn,
            )

            matched_indices = np.where(ious > iou_threshold)
            if matched_indices[0].size == 0:
                return None
            
            segment_idx = matched_indices[0]
            detection_idx = matched_indices[1]

            sorted_indices = np.argsort(-ious[segment_idx, detection_idx])  # Negative for descending sort
            segment_idx = segment_idx[sorted_indices]
            detection_idx = detection_idx[sorted_indices]

            unique_detections, first_occurrences = np.unique(detection_idx, return_index=True)
            segment_idx = segment_idx[first_occurrences]
            detection_idx = detection_idx[first_occurrences]
        
            detections = self.detections[detection_idx]
            if self.segments.object_length is not None:
                detections.object_length = self.segments.object_length[segment_idx]
               
            if self.segments.uid is not None:
                detections.uid = self.segments.uid[segment_idx]
            
            if self.segments.xyn is not None:
                detections.xyn = self.segments[segment_idx].xyn

            self.detections = detections
        except Exception as err:
            raise ValueError(f'Error in checking objects problematic: {err}')
            
        return detections
    
    def is_long(self, threshold: float) -> 'Detections':
        try:
            return self.detections[self.detections.object_length >= threshold]
            
        except Exception as err:
            raise ValueError(f'Error in check objects size: {err}')
    
    def severtiy_level(self, mapping_key:str, mapping_threshold:list):
        try:
            if not mapping_key in SEVERITY_MAP:
                logging.warning(
                    f"⚠️  given key: {mapping_key} not found in SEVERITY LEVEL Mapping, expected keys: {list(SEVERITY_MAP.keys())}" + \
                        "\nHowever key: object_length is assigned !"
                    )
                
                mapping_key = 'object_length'
            
            mapping = SEVERITY_MAP.get(mapping_key)
            if not mapping:
                return self.detections
            
            self.detections.class_id = np.array([mapping(o, mapping_threshold) for o in self.detections.object_length])
            return self.detections
            
        except Exception as err:
            raise ValueError(f'Error in assigning severity level map: {err}')

    def save(self, cv_image:np.ndarray):
        try:
            event_uid = str(uuid.uuid4())
            filename = (
                f"{sensorbox.plant_entity.entity_type.tenant.tenant_name}_"
                f"{sensorbox.plant_entity.entity_uid}_"
                f"{sensorbox.sensor_box_location}_"
                f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_"
                f"{event_uid}.jpg"
            )
            
            image = Image(
                image_id=event_uid,
                image_name=filename,
                image_format='JPEG',
                timestamp=datetime.now(tz=timezone.utc),
                sensorbox=sensorbox,
            )
            
            _, buffer = cv2.imencode('.jpg', cv_image)
            image_binary = buffer.tobytes()
            
            image.image_file.save(
                filename,
                ContentFile(image_binary)
                )
            image.save()
            
            for i, xyxyn in enumerate(self.detections.xyxyn):
                Impurity.objects.create(
                    image=image,
                    object_uid=self.detections.uid[i],
                    timestamp=datetime.now(tz=timezone.utc),
                    confidence_score=self.detections.confidence[i],
                    class_id=self.detections.class_id[i],
                    object_length=self.detections.object_length if self.detections.object_length is not None else None,
                    object_coordinates={
                        "xyxyn": xyxyn.tolist(),
                        "xyn": self.detections.xyn[i] if self.detections.xyn is not None else None
                        },
                )
                        
        except Exception as err:
            raise ValueError(f"Error saving impurity: {err}")
        
        
    