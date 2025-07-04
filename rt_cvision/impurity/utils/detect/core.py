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
from common_utils.policy.core import ImpurityClassifier
from common_utils.duplicate_tracker.core import DuplicateTracker
from common_utils.policy.utils import IMPURITY_RULES, map_attributes
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
        
        self.classifier = ImpurityClassifier(rules=IMPURITY_RULES)
        self.tracker = DuplicateTracker(
            buffer_size=self.config.get('buffer-size', 1000), 
            iou_threshold=self.config.get('iou-threshold', 0.4), 
            expiry_minutes=self.config.get("expiry-time", 10)
        )
        self.models = []
        for det_config in self.detections_models:
            if not det_config.get('active', True):
                continue

            det_name = det_config['name']
            model_cls = load_inference_module(
                config=det_config
            )
            self.models.append(
                {
                    "name": det_name,
                    "mapping": det_config.get("mapping"),
                    "model": model_cls(
                        weights=det_config["weights"],
                        device=det_config.get("device") or device,
                        config=det_config,
                    ),
                }
            )

    def infer(self, model, image: np.ndarray, confidence_threshold:Optional[float] = 0.25):
        if not self.activate_tracking:
            return model.predict(image, confidence_threshold)
        else:
            return model.track(image, confidence_threshold)
    
    def attribues(self, det_model:Dict, detections:Detections):
        if not det_model.get("mapping"):
            detections.data.update(
                {
                    "attributes": [[f"{det_model['type']}:{str(class_name).lower()}"] for class_name in detections.data["class_name"]]
                }
            )
        else:
            detections.data.update(
                {
                    "attributes": [[f"{det_model['mapping'][cls_id]}"] for cls_id in detections.class_id.astype(int)]
                }
            )

        return detections
    
    def classify(self, detections:Detections):
        outputs = []
        indexes = []
        for i in range(len(detections)):
            attrs = detections.data['attributes'][i]
            attrs_dict = map_attributes(attrs)
            context = self.classifier.classify(
                attributes=attrs_dict,
            )
            outputs.append(context)

            if "impurity_type" in context and context['impurity_type']:
                indexes.append(i)
        
        detections.data.update({"context": outputs})

        return detections, cast(Detections, detections[indexes])

    def check_duplicate(self, detections:Detections):
        try:
            unique_indices = []
            for detection_idx, xyxy in enumerate(detections.xyxyn):
                temp_det = {"bbox": xyxy, "timestamp": time.time()}
                if self.tracker.add_detection(temp_det):
                    logging.info(f"[Detection Model] Detection {detection_idx} is unique and added to the buffer.")
                    unique_indices.append(detection_idx)
                else:
                    logging.info(f"[Detection Model] Detection {detection_idx} is a duplicate.")
            
            detections = cast(Detections, detections[unique_indices])
        except Exception as err:
            logging.error(f'[Detection Model] Unexpected Error while checking duplicate: {err}')
        
        return detections

    def run(self, image:np.ndarray, segments:Detections, confidence_threshold:Optional[float] = 0.25):
        try:
            enriched_segments = segments
            assert not image is None, f'Image is None'
            assert self.models, f'No Models is provided'

            start_time = time.time()
            for det_model in self.models:
                model = det_model['model']
                detections = self.infer(model=model, image=image, confidence_threshold=confidence_threshold)
                detections = detections.with_nms()
                detections.object_length = np.zeros(len(detections))
                detections.object_area = detections.box_area
                detections.uid = np.array([str(uuid.uuid4()) for _ in range(len(detections))])
                detections = self.attribues(det_model, detections)
                logging.info(f"[Detection Model] {det_model['name']}: {len(detections)} instances")

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