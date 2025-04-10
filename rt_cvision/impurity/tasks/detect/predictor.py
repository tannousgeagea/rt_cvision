import logging
from common_utils.model.base import BaseModels
from common_utils.detection.core import Detections
from configure.client import config_manager
parameters = config_manager.params.get('impurity')

model = BaseModels(
    weights=parameters.get('weights'), task='impurity', mlflow=parameters.get('mlflow', {}).get('active', False)
)

conf = parameters.get('conf', 0.25)

def predict(image):
    detections = Detections.from_dict({})
    try:
        assert not image is None, f'Image is None'
        assert not model is None, f'Model is None'
        detections = model.classify_one(image=image, conf=float(conf), mode='detect')
        # detections = Detections.from_dict(results=results)
    except Exception as err:
        logging.error(f'Unexpected Error in Segmentation: {err}')
    
    return detections
