import logging
from common_utils.model.base import BaseModels
from common_utils.detection.core import Detections
from configure.client import config_manager
from common_utils.filters.core import FilterEngine

parameters = config_manager.params.get('segmentation')

model = BaseModels(
    weights=parameters.get('weights'), task='segments', mlflow=parameters.get('mlflow', {}).get('active', False),
)

roi = parameters.get('roi')
filter_config = parameters.get('filter_config')
filter_engine = FilterEngine()
if filter_config:
    for obj_type, filter_model in filter_config.items():
        filter_engine.add_model(
            object_type=obj_type,
            detection_model=filter_model["model_path"],
            mlflow=filter_model["mlflow"]
        )
        
def predict(image):
    detections = Detections.from_dict({})
    try:
        assert not image is None, f'Image is None'
        assert not model is None, f'Model is None'
        
        h, w, _ = image.shape
        if roi:
            roi_pixels = [(int(x * w), int(y * h)) for x, y in roi]
            x_coords, y_coords = zip(*roi_pixels)
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)
            c_image = image[y_min:y_max, x_min:x_max]
        else:
            c_image = image.copy()
        
        results = model.classify_one(image=c_image, conf=float(parameters.get('conf')), mode=parameters.get('mode'))
        detections = Detections.from_dict(results=results)
        if roi:
            detections = detections.adjust_to_roi(
                offset=(x_min, y_min),
                crop_size=c_image.shape[:2],
                original_size=image.shape[:2],
            )
            
        if filter_config:
            filtered_results = filter_engine.filter_objects(
                image=image,
                segmentation_results=detections,
                filter_types=list(filter_config.keys()),
            )
            detections = detections[filtered_results]
        
    except Exception as err:
        logging.error(f'Unexpected Error in Segmentation: {err}')
    
    return detections