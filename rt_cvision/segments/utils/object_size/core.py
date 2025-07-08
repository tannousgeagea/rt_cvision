import logging
import numpy as np
from common_utils.log import Logger
from typing import List, Tuple, Optional
from common_utils.object_size.core import ObjectSizeBase
from common_utils.detection.core import Detections

class ObjectSizeEst(ObjectSizeBase):
    def __init__(self) -> None:
        self.logger = Logger(name="Object Size Est", level=logging.DEBUG)
        super().__init__()
        
    def execute(self, detections: Detections, input_shape:Tuple, correction_factor:Optional[float]=0.001):
        try:
            index, object_length, object_area = self.compute_object_length_bbox(
                bboxes=detections.xyxyn,
                input_shape=input_shape,
                correction_factor=correction_factor,
            )
        
            detections = detections[index]
            detections.object_length = np.array(object_length)
            detections.object_area = np.array(object_area)
            
        except Exception as err:
            self.logger.error(f"Error executing object size estimation: {err}")
        
        return detections
        