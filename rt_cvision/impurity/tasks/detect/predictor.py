import logging
from common_utils.model.base import BaseModels
from common_utils.detection.core import Detections


model = BaseModels(
    weights='/home/appuser/data/models/impurity/brewa-blu-sensorbox01-conveyor01_problematic_cls_v06.pt', task='impurity',
)

def predict(image):
    detections = Detections.from_dict({})
    try:
        assert not image is None, f'Image is None'
        assert not model is None, f'Model is None'
        results = model.classify_one(image=image, conf=0.25, mode='detect')
        detections = Detections.from_dict(results=results)
    except Exception as err:
        logging.error(f'Unexpected Error in Segmentation: {err}')
    
    return detections
