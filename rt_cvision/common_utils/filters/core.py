import numpy as np
from typing import List, Dict, Any
from common_utils.model.base import BaseModels

class FilterEngine:
    def __init__(self):
        self.detection_models = {}

    def add_model(self, object_type: str, detection_model: str, mlflow:bool=False):
        """Add a new detection model for a specific object type."""
        self.detection_models[object_type] = BaseModels(weights=detection_model, mlflow=mlflow)

    def filter_objects(self, image: Any, segmentation_results: List[Dict], filter_types: List[str]) -> List[Dict]:
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
                detection_results = self.detection_models[obj_type].classify_one(
                    image=image,
                    conf=0.15,
                    mode="detect",
                )
                unwanted_rois.extend([det for det in detection_results['xyxyn']])
        
        if not unwanted_rois:
            return range(len(segmentation_results))
        
        filtered_results = []
        for i, segment in enumerate(segmentation_results.xyxyn):
            center = self._get_center(segment)
            keep = True
            for roi in unwanted_rois:
                if self._is_within(center, roi):
                    keep = False
                    break
            if keep:
                filtered_results.append(i)

        return filtered_results

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
