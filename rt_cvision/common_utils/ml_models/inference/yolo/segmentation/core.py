
import cv2
import os
from PIL import Image
import numpy as np
from ultralytics import YOLO
from common_utils.detection.core import Detections
from common_utils.ml_models.inference.base import BaseInferencePlugin
from common_utils.model.mlflow_model.core import pull


class YOLOSegInferencePlugin(BaseInferencePlugin):
    def __init__(self, weights, device, config=None):
        super().__init__(weights, device, config)
        self.model = self.init_model()
        
    def init_model(self):
        if self.config.get("mlflow"):
            return pull(self.weights).unwrap_python_model()
        
        if not os.path.exists(self.weights):
            raise FileExistsError(f"Model not found: {self.weights}")

        return YOLO(self.weights)

    def predict(self, image: np.ndarray, confidence_threshold:float=0.25) -> Detections:

        results = self.model.infer(image, conf=confidence_threshold)
        detections = Detections.from_ultralytics(
            ultralytics_results=results[0]
        )

        return detections
    

    def track(self, image: np.ndarray, confidence_threshold:float=0.25) -> Detections:

        results = self.model.track(image, conf=confidence_threshold, persist=True)
        detections = Detections.from_ultralytics(
            ultralytics_results=results[0]
        )

        return detections