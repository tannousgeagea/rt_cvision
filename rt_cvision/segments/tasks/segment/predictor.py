import pytz
import time
import torch
import logging
from datetime import datetime
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
            mlflow=filter_model["mlflow"],
            conf_threshold=filter_model.get('conf_threshold', 0.25),
        )

# Activation time parameters
start_time = parameters.get('start_time', '00:00')
end_time = parameters.get('end_time', '23:59')
timezone = parameters.get('timezone', 'UTC')

def is_within_active_time():
    """Check if current local time is within active time window."""
    try:
        local_tz = pytz.timezone(timezone)
        current_time = datetime.now(local_tz).strftime("%H:%M")
        return start_time <= current_time <= end_time
    except Exception as e:
        logging.error(f"Error checking active time window: {e}")
        return False

def predict(image):
    if not is_within_active_time():
        logging.info(f"Segmentation is inactive outside of {start_time} - {end_time}")
        return Detections.from_dict({})
    
    detections = Detections.from_dict({})
    try:
        assert not image is None, f'Image is None'
        assert not model is None, f'Model is None'
        
        start_exe_time = time.time()
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
        detections = model.classify_one(image=c_image, conf=float(parameters.get('conf')), mode=parameters.get('mode'))
        print(f"3. Total Inference Time: {round((time.time() - start_inf) * 1000, 2)} ms")
        # detections = Detections.from_dict(results=results)
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
        
        print(f"4. Total Prediction Time: {round((time.time() - start_exe_time) * 1000, 2)} ms")
        
        torch.cuda.empty_cache()
        max_memory_usage = torch.cuda.max_memory_allocated() / (1024 * 1024)
        reserved_memory = torch.cuda.max_memory_reserved() / (1024 * 1024)

        print(f'Memory Usage: {max_memory_usage} mb')
        print(f'Reserved Memory Usage: {reserved_memory} mb')
    except Exception as err:
        logging.error(f'Unexpected Error in Segmentation: {err}')
    
    return detections