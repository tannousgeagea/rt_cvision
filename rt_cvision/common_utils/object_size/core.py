import os
import cv2
import logging
import numpy as np
from typing import Union, List, Optional, Tuple
from scipy.spatial.distance import cdist, pdist
from common_utils.object_size.utils import (
    farthest_points,
    shrink_contour,
)

class ObjectSizeBase:
    def __init__(self) -> None:
        pass
    
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
                    _, cnt = shrink_contour(cnt, input_shape, iterations=3)

                object_length = self.compute_object_length(
                    contour=cnt, 
                    correction_factor=correction_factor
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

        object_lengths = np.maximum(widths, heights) * correction_factor
        object_areas = (widths * heights) / (width * height)
        indices = np.arange(bboxes.shape[0])
        return indices.tolist(), object_lengths.tolist(), object_areas.tolist()
    
if __name__ == "__main__":
    xyn = [
    (0.202366428125, 0.265762784375), (0.21614583125000003, 0.2642702359375),
    (0.226903396875, 0.23596001875), (0.2354272765625, 0.22743613906250001),
    (0.23769383125000001, 0.202000084375), (0.228217453125, 0.1921685453125),
    (0.2112847359375, 0.18944229375), (0.2049616734375, 0.183829553125),
    (0.19226624375, 0.18107865), (0.1838409703125, 0.17265337812499998),
    (0.17314914374999998, 0.189664159375), (0.1646745671875, 0.1897134625),
    (0.16147194843749998, 0.193969240625), (0.1656270640625, 0.2080688828125),
    (0.165512021875, 0.22784289375000003), (0.16337694375, 0.2306800796875),
    (0.1770577390625, 0.2461366828125), (0.1833972375, 0.24892456406250002),
    (0.189703865625, 0.2573621625), (0.202366428125, 0.265762784375)
    ]
    
    object_size_est = ObjectSizeBase()
    object_size_est.process(xyn=[xyn], input_shape=(1536, 2048), correction_factor=0.001, erode=True)
    
    
