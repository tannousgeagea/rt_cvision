
import logging
import numpy as np
from typing import Union
from typing import Optional, Dict, List
from common_utils.detection.core import Detections
from common_utils.detection.utils import box_iou_batch
from impurity.tasks.severity_level.assign_severity_level import SEVERITY_LEVEL_MAP, SEVERITY_LEVEL_MAP_vectorized

def is_object_problematic(
    detections:Detections, 
    segments:Dict, 
    iou_threshold:float=0.45, 
    mapping_key:str='object_length', 
    mapping_threshold:list=[0, 1., 1.5, 2.],
    ):
    
    problematic_detection = segments.copy()
    try:
        ious = box_iou_batch(
            np.array(segments.get('xyxyn')),
            detections.xyxyn,
        )
        
        ious.sort() 
        index = np.where(ious > iou_threshold)
        for i, j in zip(index[0], index[1]):
            segments['class_id'][i] = detections.class_id[j]
        
        problematic_detection = (ious > iou_threshold).any(axis=1)
        problematic_detection = {
            key: [
                value[i] for i in range(len(value))
                if problematic_detection[i]
            ] 
            for key, value in segments.items()
            if isinstance(value, list)
        }
        
        if not len(problematic_detection["class_id"]):
            return problematic_detection

        if mapping_key == 'class_id':
            problematic_detection = SEVERITY_LEVEL_MAP_vectorized(
                objects=problematic_detection, key="class_id", thresholds=[1, 2, 3]
                )
        else:
            problematic_detection = SEVERITY_LEVEL_MAP(
                objects=problematic_detection, key=mapping_key, thresholds=mapping_threshold
            )
        
    except Exception as err:
        logging.error(f'Error in checking objects problematic: {err}')
        
    return problematic_detection