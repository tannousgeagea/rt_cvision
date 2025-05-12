import logging
import numpy as np
from typing import List, Optional
from common_utils.object_size.core import ObjectSizeBase
from common_utils.object_size.zones import ZoneConfig, Zone

class ObjectSizeEst(ObjectSizeBase):
    def __init__(self, zones: Optional[List[List[float]]] = None) -> None:
        zone_config = None
        logging.info(f"Zones: {zones}")
        if zones:
            zone_objects = [Zone(*z) for z in zones]  # unpack each list as Zone args
            zone_config = ZoneConfig(zone_objects)
        super().__init__(zone_config=zone_config)
        
    def execute(self, objects, input_shape, correction_factor:float=0.001):
        try:
            xyxyn = objects.get('xyxyn')
            if not xyxyn:
                logging.warning("No bounding boxes (xyxyn) provided.")
                return objects
            
            index, object_length, object_area = self.compute_object_length_bbox(
                bboxes=np.array(objects.get('xyxyn')),
                input_shape=input_shape,
                correction_factor=correction_factor,
            )
            
            objects = {
                key: [value[i] for i in index]
                for key, value in objects.items()
                if isinstance(value, list) and len(value) > max(index)
            }
            
            objects['object_length'] = object_length
            objects['object_area'] = object_area
            
        except Exception as err:
            logging.error(f"Error executing object size estimation: {err}")
        
        return objects
        