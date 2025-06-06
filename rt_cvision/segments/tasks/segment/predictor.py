import time
import torch
import logging
from common_utils.model.base import BaseModels
from common_utils.detection.core import Detections
from configure.client import ConfigManager
from common_utils.filters.core import FilterEngine

config_manager = ConfigManager()

model = BaseModels(
    weights=config_manager.segmentation.weights, task='segments', mlflow=config_manager.segmentation.mlflow.get('active', False),
)

roi = getattr(config_manager.segmentation, "roi", None)
filter_config = getattr(config_manager.segmentation, "filter_config", None)
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
        
        start_time = time.time()
        h, w, _ = image.shape
        if roi:
            roi_pixels = [(int(x * w), int(y * h)) for x, y in roi]
            x_coords, y_coords = zip(*roi_pixels)
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)
            c_image = image[y_min:y_max, x_min:x_max]
        else:
            c_image = image.copy()
        
        start_inf = time.time()
        detections = model.classify_one(image=c_image, conf=float(config_manager.segmentation.conf), mode=config_manager.segmentation.mode, is_json=False)
        print(f"3. Total Inference Time: {round((time.time() - start_inf) * 1000, 2)} ms")
        
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
        
        print(f"4. Total Prediction Time: {round((time.time() - start_time) * 1000, 2)} ms")

        torch.cuda.empty_cache()
        max_memory_usage = torch.cuda.max_memory_allocated() / (1024 * 1024)
        reserved_memory = torch.cuda.max_memory_reserved() / (1024 * 1024)

        print(f'Memory Usage: {max_memory_usage} mb')
        print(f'Reserved Memory Usage: {reserved_memory} mb')
    except Exception as err:
        logging.error(f'Unexpected Error in Segmentation: {err}')
    
    return detections