import os
import cv2
import logging
import numpy as np
from common_utils.object_size.zones import ZoneConfig, Zone
from typing import Union, List, Optional, Tuple
from scipy.spatial.distance import cdist, pdist
from common_utils.object_size.utils import (
    farthest_points,
    shrink_contour,
)

class ObjectSizeBase:
    def __init__(self, zone_config: Optional[ZoneConfig] = None) -> None:
        self.zone_config = zone_config
    

    def set_grid_zones(self, rows: int, cols: int, base_correction: float, variation_fn=None):
        zones = []
        for row in range(rows):
            for col in range(cols):
                x_start = col / cols
                x_end = (col + 1) / cols
                y_start = row / rows
                y_end = (row + 1) / rows
                factor = variation_fn(row, col) if variation_fn else base_correction
                zones.append(Zone(x_start, x_end, y_start, y_end, factor))
        self.zone_config = ZoneConfig(zones)

    def compute_object_length(self, contour:np.ndarray, correction_factor:float):
        object_length = 0.
        try:
            farthest_pts = farthest_points(contour=contour)
            object_length = pdist(
                [
                    farthest_pts[0], 
                    farthest_pts[1]]
                )[0]
            
        except Exception as err:
            logging.error(f"Unexpected Error in computing object length: {err}")

        return object_length * correction_factor

    def process(
        self,
        xyn:Union[List, np.ndarray],
        input_shape:Union[List, Tuple],
        correction_factor:float,
        eps:Optional[float]=0.95,
        erode:Optional[bool]=False,
        erode_iterations:Optional[int]=3,
    ):
        """
        Process an input image for object size estimation.

        Args:
            - xyn (list, optional): List of polygons representing object shapes.
            - input_shape (list, tuple): A tuple representing the dimensions (height, width) of the area containing the contour.
            - correction_factor (float): factor to convert pixel to metric wise. 
            - eps (float, optional): Factor to adjust object length estimation. Defaults to 0.85.
            - erode (bool, optianl): boolen if set to True, shrink the contour.
            - erode_iterations (int, optional): when erode set to True, iterations of the erosion

        Returns:
            - tuple: list of object_index, object lenght, object_area
        """
        total_object_length = []
        total_object_area = []
        index = []
        try:
            if not xyn:
                return index, total_object_length, total_object_area
            
            for i, cnt in enumerate(xyn):
                cnt = np.array(cnt)

                if not len(cnt):
                    continue
                
                cnt =  np.array([(int(x * input_shape[1]), int(y * input_shape[0])) for x, y in cnt])
                if erode:
                    _, cnt = shrink_contour(cnt, input_shape, iterations=erode_iterations)

                avg_x = int(np.mean(cnt[:, 0]))
                avg_y = int(np.mean(cnt[:, 1]))

                zone_correction = (
                    self.zone_config.get_correction_factor(avg_x, avg_y, input_shape[1], input_shape[0], baseline=correction_factor)
                    if self.zone_config else correction_factor
                )

                object_length = self.compute_object_length(
                    contour=cnt, 
                    correction_factor=zone_correction
                )
                
                object_area = cv2.contourArea(cnt) / (input_shape[0] * input_shape[1])  
                total_object_area.append(object_area)
                total_object_length.append(object_length)
                index.append(i)

        except Exception as err:
            logging.error(f"Unexpected Error in processing contour in computing object size: {err}")

        return index, total_object_length, total_object_area

    def compute_object_length_bbox(self, bboxes: np.ndarray, input_shape, correction_factor: float):
        """
        Process an array of normalized bounding boxes using vectorized NumPy operations
        to compute object sizes.

        Args:
            bboxes (np.ndarray): Array of shape (N, 4) where each row is [xmin, ymin, xmax, ymax]
                                 with normalized coordinates (values between 0 and 1).
            input_shape (Tuple[int, int]): Image dimensions (height, width).
            correction_factor (float): Factor to convert pixel measurements to metric.

        Returns:
            tuple: Three lists containing:
                - Object indices,
                - Object lengths (after applying the correction factor),
                - Object areas (normalized by the image area).
        """

        if bboxes is None or bboxes.shape[0] == 0:
            logging.info("No bounding boxes provided.")
            return [], [], []  # Return empty lists if no bboxes
        
        height, width, _ = input_shape
        xmin = bboxes[:, 0] * width
        ymin = bboxes[:, 1] * height
        xmax = bboxes[:, 2] * width
        ymax = bboxes[:, 3] * height

        widths = xmax - xmin
        heights = ymax - ymin

        centers_x = ((xmin + xmax) / 2).astype(int)
        centers_y = ((ymin + ymax) / 2).astype(int)

        if self.zone_config:
            correction_factors = np.array([
                self.zone_config.get_correction_factor(x, y, width, height, baseline=correction_factor)
                for x, y in zip(centers_x, centers_y)
            ])
        else:
            correction_factors = np.full_like(centers_x, correction_factor, dtype=float)

        object_lengths = np.maximum(widths, heights) * correction_factors
        object_areas = (widths * heights) / (width * height)
        indices = np.arange(bboxes.shape[0])
        return indices.tolist(), object_lengths.tolist(), object_areas.tolist()
    