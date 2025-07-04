import cv2
import logging
import numpy as np
from typing import List, Dict, Tuple
from common_utils.annotate.color import Color
from scipy.spatial.distance import cdist, pdist

def farthest_points(contour):
    """
    Finds the farthest points in a contour.

    Parameters:
    - contour: a list of 2D points representing the contour.

    Returns:
    - a tuple containing the two farthest points in the contour.
    """
    # Convert the contour to a NumPy array
    contour = np.array(contour).squeeze()
    
    try:
        # Compute the pairwise distances between all points in the contour
        dists = cdist(contour, contour)

        # Find the indices of the two points with the maximum distance
        i, j = np.unravel_index(np.argmax(dists), dists.shape)

        # Return the two farthest points as a tuple
        return tuple(contour[i]), tuple(contour[j])

    except Exception as err:
        raise ValueError(f"Error in computing farthest points in a contour: {err}")


def shrink_contour(contour, input_shape, iterations:int=1):
    """
    Reduces the size of a contour by performing erosion.

    Args:
        contour (list of tuples): The contour represented as a list of (x, y) coordinates.
        input_shape (tuple): A tuple representing the dimensions (height, width) of the area containing the contour.
        iterations (int): The number of iterations to reduce the contour size.

    Returns:
        list of tuples: The inner contour represented as a list of (x, y) coordinates.
    """
    success = False
    inner_contour = contour
    try:
        contour = contour.astype(np.int32)
        
        # Create a blank image of the same size as the original contour
        mask = np.zeros(shape=(input_shape[0], input_shape[1]), dtype='uint8')

        # Draw the original contour on the mask
        cv2.drawContours(mask, [contour], 0, 255, thickness=cv2.FILLED)
        
        # Perform erosion on the mask to reduce the contour size
        eroded = cv2.erode(mask, None, iterations=iterations)

        # Find the contours in the eroded image
        contours, _ = cv2.findContours(eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Select the inner contour
        if len(contours):
            success=True
            inner_contour = max(contours, key=cv2.contourArea)
            
    except Exception as err:
        logging.error(f"Error in shrinking contour: {err}")


    return success, inner_contour


def extract_size_threshold(object_length_threshold: List[Dict]) -> Tuple[List[str], List[Tuple[int, int, int]], List[float]]:
    labels, colors, thresholds = [], [], []
    
    for threshold in object_length_threshold:
        try:
            min_val = threshold["min"]
            max_val = threshold.get("max", None)
            label = f"{min_val} - {max_val}" if max_val is not None else f"> {min_val}"
            labels.append(label)
            thresholds.append(min_val)
            colors.append(Color.from_hex(threshold["color"]).as_bgr())
        except Exception as e:
            logging.warning(f"Skipping invalid threshold entry: {threshold}, Error: {e}")
    
    return labels, colors, thresholds


def get_size_threshold(object_length_threshold: List[Dict]) -> Tuple[List[str], List[Tuple[int, int, int]], List[float]]:
    try:
        return extract_size_threshold(object_length_threshold)

    except Exception as err:
        logging.error(f"[Size Threshold]: Error in config: {err}. Using fallback thresholds.")
        fallback_threshold = [
            {"min": 0,   "max": 0.5, "label": "Small",      "color": "#00ff00"},
            {"min": 0.5, "max": 1.0, "label": "Medium",     "color": "#ffe933"},
            {"min": 1.0, "max": 1.5, "label": "Large",      "color": "#ffa833"},
            {"min": 1.5, "max": None,"label": "Very Large", "color": "#ff3333"},
        ]
        return extract_size_threshold(fallback_threshold)