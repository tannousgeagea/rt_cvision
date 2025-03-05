import logging
import numpy as np
from common_utils.object_size.core import ObjectSizeBase

class ObjectSizeEst(ObjectSizeBase):
    def __init__(self) -> None:
        super().__init__()
        
    def execute(self, objects, input_shape, correction_factor:float=0.001):
        try:
            index, object_length, object_area = self.compute_object_length_bbox(
                bboxes=np.array(objects.get('xyxyn')),
                input_shape=input_shape,
                correction_factor=correction_factor,
            )
            
            objects = {
                key: [value[i] for i in index]
                for key, value in objects.items()
                if isinstance(value, list)
            }
            
            objects['object_length'] = object_length
            objects['object_area'] = object_area
            
        except Exception as err:
            logging.error(f"Error executing object size estimation: {err}")
        
        return objects
        