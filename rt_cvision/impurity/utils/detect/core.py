import time
import pytz
import uuid
import hashlib
import numpy as np
import logging
import torch
from copy import deepcopy
from datetime import datetime
from typing import Dict, List, Optional, cast, Tuple
from common_utils.ml_models import load_inference_module
from common_utils.detection.core import Detections
from common_utils.detection.assign import enrich_segments_with_detections

device = "gpu" if torch.cuda.is_available() else "cpu"

class DetectionModel:
    def __init__(self, config:Dict) -> None:

        self.map_tracker_id_2_object_uid = {}
        self.config = config
        self.activate_tracking = config.get("activate_tracking", False)
        self.detections_models = self.config.get("detections_models")
        if not self.detections_models:
            raise ValueError(f"detection_models is required !")
        
        self.models = {}
        for det_config in self.detections_models:
            det_name = det_config['name']
            model_cls = load_inference_module(
                config=det_config
            )

            self.models[det_name] = model_cls(
                weights=det_config["weights"],
                device=det_config.get("device") or device,
                config=det_config,
            )

    def infer(self, model, image: np.ndarray, confidence_threshold:Optional[float] = 0.25):
        if not self.activate_tracking:
            return model.predict(image, confidence_threshold)
        else:
            return model.track(image, confidence_threshold)
        
    def run(self, image:np.ndarray, segments:Detections, confidence_threshold:Optional[float] = 0.25):
        try:
            enriched_segments = segments
            assert not image is None, f'Image is None'
            assert self.models, f'No Models is provided'

            start_time = time.time()
            for model_name, det_model in self.models.items():
                detections = self.infer(model=det_model, image=image, confidence_threshold=confidence_threshold)
                detections = detections.with_nms()
                detections.object_length = np.zeros(len(detections))
                detections.object_area = detections.box_area
                logging.info(f"[Detection Model] {model_name}: {len(detections)} instances")

                enriched_segments = enrich_segments_with_detections(
                    segments=enriched_segments, detections=detections, iou_threshold=0.4, confidence_threshold=0.7
                    )
                
            logging.info(f"[Detection Model] Total Prediction Time: {round((time.time() - start_time) * 1000, 2)} ms")

            torch.cuda.empty_cache()
            max_memory_usage = torch.cuda.max_memory_allocated() / (1024 * 1024)
            reserved_memory = torch.cuda.max_memory_reserved() / (1024 * 1024)

            logging.info(f'[Detection Model] Memory Usage: {max_memory_usage} mb')
            logging.info(f'[Detection Model] Reserved Memory Usage: {reserved_memory} mb')

            return enriched_segments
        except Exception as err:
            raise ValueError(f'[Detection Model] Unexpected Error in Detecion Model: {err}')