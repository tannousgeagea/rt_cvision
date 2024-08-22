import cv2
import logging
import numpy as np
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

