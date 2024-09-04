import os
import cv2
import time
import logging
import numpy as np
from ultralytics import YOLO
from pathlib import Path

base_dir = Path(__file__).parent

print(base_dir)

class BaseModels:
    def __init__(
            self,
            config_params=None,
            weights=None,
            task=None,

    ):
        self.weights = weights
        self.task = task

        if not config_params is None:
            self.weights = config_params['weights']

        self.model = self.init_model()

    def init_model(self):
        if not os.path.exists(self.weights):
            logging.warning("⚠️ Warning: Model weights %s does not exists" % self.weights)
            if not os.path.exists(f"{base_dir}/weights/base.{self.task}.pt"):  
                return None
            
            logging.info(f"Loading base model: {base_dir}/weights/base.{self.task}.pt")
            return YOLO(f"{base_dir}/weights/base.{self.task}.pt")
            
        
        logging.info(f'Model weights: {self.weights} successfully loaded! ✅')
        return YOLO(self.weights)

    def classify_one(self, image, conf=0.25, mode='detect', classes=None):
        final_results = {}
        if self.model:
            results = self.model.track(image, persist=True, conf=conf, classes=classes) if mode=='track' else self.model.predict(image, conf=conf, classes=classes)
            
            final_results = self.write_result(final_results, 'class_names', results[0].names)
            if not results[0].probs is None:
                final_results = self.write_result(final_results, 'probabilities', results[0].probs.data.cpu().numpy().tolist())
            
            if not results[0].boxes is None:
                final_results = self.write_result(final_results, 'xyxy', results[0].boxes.xyxy.cpu().numpy().astype(int).tolist())
                final_results = self.write_result(final_results, 'xyxyn', results[0].boxes.xyxyn.cpu().numpy().tolist())
                final_results = self.write_result(final_results, 'confidence_score', results[0].boxes.conf.cpu().numpy().tolist())
                final_results = self.write_result(final_results, 'class_id', results[0].boxes.cls.cpu().numpy().astype(int).tolist())
                if not results[0].boxes.id is None:
                    final_results = self.write_result(final_results, 'tracker_id', results[0].boxes.id.cpu().numpy().astype(int).tolist())
            
            if not results[0].masks is None:
                final_results = self.write_result(final_results, 'xy', results[0].masks.xy)
                final_results = self.write_result(final_results, 'xyn', results[0].masks.xyn)
            else:
                final_results = self.write_result(final_results, 'xy', [])
                final_results = self.write_result(final_results, 'xyn', [])
                
        return final_results
    
    def write_result(self, result, key, value):
        if key not in result.keys():
            result[key] = None
        
        result[key] = value

        return result