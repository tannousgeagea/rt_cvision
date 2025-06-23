import logging
import numpy as np
from common_utils.model.base import BaseModels
from common_utils.detection.core import Detections
from common_utils.detection.convertor import (
    copy_and_paste
)
from configure.client import config_manager
parameters = config_manager.params.get('impurity')

model = BaseModels(
    weights=parameters.get('material_cls_weights', ""), task='material_cls', mlflow=parameters.get('mlflow', {}).get('active', False)
)

def filter_from_material(objects:dict, image:np.ndarray, filter_materials:list):
    try:

        if not model.model:  # Ensure model is loaded before running classification
            logging.warning("⚠️ Model is not loaded. Skipping classification.")
            return objects
            
        if not 'xy' in objects:
            return objects

        objects['material'] = [None for _ in range(len(objects['xy']))]
        for i, polygon in enumerate(objects.get('xy')):
            xy = np.array((polygon)).astype(np.int32)

            background = copy_and_paste(
                img=image,
                polygon=xy,
                kernel=np.ones((5, 5), np.uint8)
            )

            res = model.classify_one(
                background, conf=0.6
            )

            detections = Detections.from_dict(res)
            if not len(detections):
                continue

            cls_id = int(detections.class_id[0])
            conf = float(detections.confidence[0])
            class_name = res.get("class_names")[cls_id]

            objects['material'][i] = class_name
        
        objects = {
            key: [
                value[i] for i in range(len(value)) if objects['material'][i] not in filter_materials
            ]
            for key, value in objects.items()
            if isinstance(value, list)
        }

        return objects

    except Exception as err:
        logging.error(f'Error in  filtering objects material: {err}')
    
    return objects
            