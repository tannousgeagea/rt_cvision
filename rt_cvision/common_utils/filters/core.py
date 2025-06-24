import logging
import numpy as np
from typing import List, Dict, Any
from common_utils.detection.core import Detections
from common_utils.model.base import BaseModels
from common_utils.ml_models import load_inference_module

class FilterEngine:
    def __init__(self):
        self.detection_models = {}

    def add_model(self, object_type: str, detection_model: str, config:dict, device:str, mlflow:bool=False, conf_threshold: float = 0.15):
        """Add a new detection model for a specific object type."""
        if "type" not in config:
            config["type"] =  "detection"

        if "framework" in config:
            config["framework"] = "yolo"

        model_plugin = load_inference_module(
            config=config
        )

        logging.info(config)
        self.detection_models[object_type] = {
            "model": model_plugin(weights=detection_model, device=device, config=config),
            "conf_threshold": conf_threshold
        }

    def filter_objects(self, image: Any, segmentation_results: Detections, filter_types: List[str]):
        """
        Filters unwanted objects from segmentation results based on center point containment in ROI.
        - image: Original image
        - segmentation_results: List of segmentation bounding boxes/masks
        - filter_types: List of objects to filter (e.g., ['crane', 'truck'])

        Returns:
            Filtered segmentation results.
        """
        unwanted_rois = []
        for obj_type in filter_types:
            if obj_type in self.detection_models:
                model_entry = self.detection_models[obj_type]
                detection_model = model_entry["model"]
                conf_threshold = model_entry["conf_threshold"]
                detection_results = detection_model.predict(
                    image=image,
                    confidence_threshold=conf_threshold,
                )
                unwanted_rois.extend([{"det": det, "filter_type": obj_type} for det in detection_results.xyxyn.tolist()])
        
        if not unwanted_rois:
            return np.arange(len(segmentation_results)), unwanted_rois
        
        filtered_results = []
        for i, segment in enumerate(segmentation_results.xyxyn):
            center = self._get_center(segment)
            keep = True
            for roi in unwanted_rois:
                if self._is_within(center, roi["det"]):
                    keep = False
                    break
            if keep:
                filtered_results.append(i)

        return np.array(filtered_results), unwanted_rois

    def _get_center(self, bbox):
        """
        Compute the center of a bounding box.
        - bbox: [x_min, y_min, x_max, y_max]

        Returns:
            (center_x, center_y)
        """
        center_x = (bbox[0] + bbox[2]) / 2
        center_y = (bbox[1] + bbox[3]) / 2
        return (center_x, center_y)

    def _is_within(self, point, roi):
        """
        Check if a point (center) is within a bounding box (ROI).
        - point: (x, y) coordinates of the center point.
        - roi: [x_min, y_min, x_max, y_max]

        Returns:
            True if the point is within the ROI, False otherwise.
        """
        x, y = point
        return (
            roi[0] <= x <= roi[2] and
            roi[1] <= y <= roi[3]
        )
