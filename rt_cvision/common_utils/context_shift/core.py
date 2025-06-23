import os
import logging
import numpy as np
from common_utils.detection.utils import box_iou_batch
from common_utils.model.base import BaseModels, Detections
from common_utils.object_size.core import ObjectSizeBase
from common_utils.filters.core import FilterEngine

class ContextShift:
    def __init__(self, model_path: str = None, mlflow:bool = False, conf_threshold:float=0.25, X:int=2, cf:float=0.001, filter_config:dict=None):
        self.model = None
        self.conf_threshold = conf_threshold
        self.object_size_est = ObjectSizeBase()
        self.X = X
        self.cf = cf
        if model_path and os.path.exists(model_path):
            try:
                self.model = self.load_model(model_path, mlflow=mlflow)
                logging.info(f"Context Shift model {model_path} loaded successfully.")
            except Exception as err:
                logging.error(f"Failed to load Severity Level 2 model: {err}")
        elif mlflow:
            self.model = self.load_model(model_path, mlflow=mlflow)
        else:
            logging.warning("Context Shift model path not provided or file does not exist.")

        self.filter_config = filter_config
        self.filter_engine = FilterEngine()
        if filter_config:
            for obj_type, filter_model in filter_config.items():
                self.filter_engine.add_model(
                    object_type=obj_type,
                    detection_model=filter_model["model_path"],
                    mlflow=filter_model["mlflow"],
                    conf_threshold=filter_model.get('conf_threshold', 0.25),
                )
    
    def load_model(self, path: str, mlflow=True):
        return BaseModels(weights=path, mlflow=mlflow)
    
    def infer(self, full_image: np.ndarray,) -> Detections:
        """
        Run the secondary detection model on the full image.
        Expected output format is a dict with:
            - 'xyxyn': list of bounding boxes in normalized format,
            - 'scores': list of confidence scores,
            - 'class_id': list of predicted class ids (for this model, we expect severe objects, e.g. level 2).
        """
        if self.model is None:
            return Detections.from_dict({})
        try:
            detections = self.model.classify_one(full_image, conf=self.conf_threshold, mode='detect')
            detections.class_id = np.array([self.X] * len(detections))
            return detections
        except Exception as err:
            logging.error(f"Error during context model inference: {err}")
            return Detections.from_dict({})
    
    def integrate(self, primary_detections: dict, full_image: np.ndarray, iou_threshold: float = 0.5) -> dict:
        """
        Integrates the secondary model detections with the primary detections.
        - Runs the secondary detector on the full image.
        - For each secondary detection:
            - If it matches (IoU > threshold) a primary detection, overwrite that detection.
            - If it doesn't match any primary detection, add it as a new detection.
        Returns: A dictionary containing the merged detections.
        """
        if not len(primary_detections.get("xyxyn", [])):
            return primary_detections

        secondary_results = self.infer(full_image)
        if not len(secondary_results):
            logging.info("No secondary detections available.")
            return primary_detections

        if self.filter_config:
            filtered_results = self.filter_engine.filter_objects(
                image=full_image,
                segmentation_results=secondary_results,
                filter_types=list(self.filter_config.keys()),
            )
            secondary_results = secondary_results[filtered_results]

        primary_boxes = np.array(primary_detections.get("xyxyn", []))
        secondary_boxes = np.array(secondary_results.xyxyn)

        ious = box_iou_batch(secondary_boxes, primary_boxes)  # Shape: [n_secondary, n_primary]
        primary_matches = np.full(len(secondary_boxes), -1)  # -1 signifies no match.
        
        for sec_idx in range(len(secondary_boxes)):
            matches = np.where(ious[sec_idx] > iou_threshold)[0]
            if matches.size > 0:
                # Choose the primary detection with the highest IoU.
                best_match = matches[np.argmax(ious[sec_idx, matches])]
                primary_matches[sec_idx] = best_match

        # Create a copy of primary detections to update.
        updated_primary = {key: list(value) for key, value in primary_detections.items() if isinstance(value, list)}
        updated_primary.update(
            {
                "context": [None for _ in range(len(updated_primary.get("xyxyn")))]
            }
        )

        logging.info(primary_matches)
        for sec_idx, match_idx in enumerate(primary_matches):
            if match_idx != -1:
                # Overwrite the matching primary detection with secondary detection details.
                updated_primary["xyxyn"][match_idx] = secondary_results.xyxyn[sec_idx].tolist()
                updated_primary["class_id"][match_idx] = secondary_results.class_id[sec_idx].tolist()
                updated_primary["context"][match_idx] = secondary_results.data.get("class_name", [])[sec_idx] if secondary_boxes.data else None
                if "severity_level" in primary_detections:
                    updated_primary["severity_level"][match_idx] = self.X + 1
                else:
                    updated_primary["severity_level"] = [self.X + 1] * len(primary_detections["xyxyn"])
        
        return updated_primary
