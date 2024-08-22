import os
import cv2
import logging
import numpy as np
from typing import Any, Iterator, List, Optional, Tuple, Union


def xyxy2xyxyn(xyxy, image_shape):
    """
    Convert bounding box coordinates from pixel format to normalized format.

    This function normalizes the bounding box coordinates based on the image dimensions. 
    The pixel coordinates (xmin, ymin, xmax, ymax) are converted to a normalized format 
    where each coordinate is represented as a fraction of the image's width or height.

    Parameters:
    - xyxy (tuple): A tuple of four integers (xmin, ymin, xmax, ymax) representing the bounding box coordinates in pixel format.
    - image_shape (tuple): A tuple of two integers (height, width) representing the dimensions of the image.

    Returns:
    - tuple: A tuple of four floats (xmin_n, ymin_n, xmax_n, ymax_n) representing the normalized bounding box coordinates.
    """
    xmin, ymin, xmax, ymax = xyxy
    return (xmin / image_shape[1], ymin / image_shape[0], xmax / image_shape[1], ymax / image_shape[0])

def normalize_boxes(boxes, image_shape):
    new_boxes = []
    for xyxy in boxes:
        new_boxes.append(xyxy2xyxyn(xyxy, image_shape))
    
    return new_boxes

def xyxyn2xyxy(xyxyn, image_shape):
    """
    Convert bounding box coordinates from normalized format to pixel format.

    This function converts the normalized bounding box coordinates back to pixel format. 
    The normalized coordinates (xmin_n, ymin_n, xmax_n, ymax_n), represented as fractions 
    of the image's width or height, are scaled back to the pixel dimensions of the image.

    Parameters:
    - xyxyn (tuple): A tuple of four floats (xmin_n, ymin_n, xmax_n, ymax_n) representing the normalized bounding box coordinates.
    - image_shape (tuple): A tuple of two integers (height, width) representing the dimensions of the image.

    Returns:
    - tuple: A tuple of four integers (xmin, ymin, xmax, ymax) representing the bounding box coordinates in pixel format.
    """
    xmin, ymin, xmax, ymax = xyxyn
    return (int(xmin * image_shape[1]), int(ymin * image_shape[0]), int(xmax * image_shape[1]), int(ymax * image_shape[0]))

def xyxy2xywh(xyxy):
    """
    Convert bounding box coordinates from (xmin, ymin, xmax, ymax) format to (x, y, width, height) format.

    Parameters:
    - xyxy (Tuple[int, int, int, int]): A tuple representing the bounding box coordinates in (xmin, ymin, xmax, ymax) format.

    Returns:
    - Tuple[int, int, int, int]: A tuple representing the bounding box in (x, y, width, height) format. 
                                 (x, y) are  the center of the bounding box.
    """
    xmin, ymin, xmax, ymax = xyxy
    w = xmax - xmin
    h = ymax - ymin
    return (xmin + w/2, ymin + h/2, w, h)

def xywh2xyxy(xywh):
    """
    Convert bounding box coordinates from (x, y, width, height) format to (xmin, ymin, xmax, ymax) format.

    This function assumes (x, y) as the center of the bounding box and calculates 
    the coordinates of the top-left corner (xmin, ymin) and the bottom-right corner (xmax, ymax).

    Parameters:
    - xywh (Tuple[float, float, float, float]): A tuple representing the bounding box in (x, y, width, height) format.

    Returns:
    - Tuple[float, float, float, float]: A tuple representing the bounding box in (xmin, ymin, xmax, ymax) format.
    """
    x, y, w, h = xywh
    return (x - w/2, y - h/2, x + w/2, y + h/2)

def poly2xyxy(poly):
   
   """
    Convert a polygon representation to an axis-aligned bounding box.

    This function takes a list of vertices (x, y) of a polygon and calculates the minimum and 
    maximum x and y coordinates. The result is a bounding box (xmin, ymin, xmax, ymax) that 
    tightly encloses the polygon.

    Parameters:
    - poly (List[Tuple[int, int]]): A list of tuples, where each tuple represents a vertex (x, y) of the polygon.

    Returns:
    - Tuple[int, int, int, int]: A tuple representing the bounding box (xmin, ymin, xmax, ymax) of the polygon.
    """
   poly = np.array(poly)
   return (min(poly[:, 0]), min(poly[:, 1]), max(poly[:, 0]), max(poly[:, 1]))


def extract_xyxy_from_txtfile(txt_file):
    """
    Extract bounding box data from a text file.

    This function reads a text file containing bounding box data. Each line in the file should 
    represent a bounding box or a polygon, starting with a class ID followed by the vertices coordinates.
    If a line contains more than 4 coordinates, it is treated as a polygon and converted to an axis-aligned
    bounding box. The function returns class IDs and bounding boxes.

    Parameters:
    - txt_file (str): The path to the text file containing the bounding box data.

    Returns:
    - A tuple of two lists, the first being class IDs and 
      the second being bounding boxes (each box either as (xmin, ymin, xmax, ymax) or as a polygon)
    """
    boxes = []
    class_ids = []
    if not os.path.exists(txt_file):
        logging.info("⚠️  Warning: %s not found !!!" %txt_file)
        return class_ids, boxes
    
    bounding_boxes_data = open(txt_file, 'r').readlines()
    if not len(bounding_boxes_data):
        logging.info('⚠️  Warning: empty file')
        return class_ids, boxes
    
    for line in bounding_boxes_data:
        parts = line.split()
        class_id = int(parts[0])
        vertices = parts[1:]
        xywh = tuple(map(float, vertices))

        if len(vertices)>4:
            polygon = [(float(vertices[i]), float(vertices[i + 1])) for i in range(0, len(vertices), 2)]
            xyxyn = poly2xyxy(poly=polygon)
        else:
            xyxyn = xywh2xyxy(xywh)
    
        class_ids.append(class_id)
        boxes.append(xyxyn)
    
    return class_ids, boxes


def extract_xy_from_txtfile(txt_file):
    """
    Extract polygon data from a text file.

    This function reads a text file containing polygon data. Each line in the file should 
    represent a polygon, starting with a class ID followed by the vertices coordinates.
    The function returns class IDs and bounding boxes.

    Parameters:
    - txt_file (str): The path to the text file containing the bounding box data.

    Returns:
    - A tuple of two lists, the first being class IDs and 
      the second being polygon coordinate (each box either as (x, y) or as a polygon)
    """
    polygons = []
    class_ids = []
    if not os.path.exists(txt_file):
        logging.info("⚠️  Warning: %s not found !!!" %txt_file)
        return class_ids, polygons
    
    polygons_data = open(txt_file, 'r').readlines()
    if not len(polygons_data):
        logging.info('⚠️  Warning: empty file')
        return class_ids, polygons
    
    
    for line in polygons_data:
        parts = line.split()
        class_id = int(parts[0])
        vertices = parts[1:]
        xy = tuple(map(float, vertices))
        xy = [(float(xy[i]), float(xy[i + 1])) for i in range(0, len(xy), 2)]
        
        class_ids.append(class_id)
        polygons.append(xy)
    
    return class_ids, polygons

def xyn2xy(xyn, image_shape):
    """
    Convert normalized polygon coordinates to actual coordinates.

    Normalized polygon coordinates are in the range [0, 1]. This function
    scales these coordinates to actual pixel coordinates based on the dimensions
    of the image or canvas.

    Parameters:
    - xyn (list of tuples): List of normalized coordinates in the format [(x1, y1), (x2, y2), ...].
    - image_shape (tuple): A tuple of two integers (height, width) representing the dimensions of the image.

    Returns:
    list of tuples: List of actual coordinates in the format [(x1, y1), (x2, y2), ...].
    """

    xy = [(int(x * image_shape[1]), int(y * image_shape[0])) for x, y in xyn]
    return xy

def xy2xyn(xy, image_shape):
    """
    Convert actual polygon coordinates to normalized coordinates.

    This function scales actual pixel coordinates of a polygon to normalized coordinates
    in the range [0, 1], which are independent of the actual dimensions of the image or canvas.

    Parameters:
    - xy (list of tuples): List of actual coordinates in the format [(x1, y1), (x2, y2), ...].
    - image_shape (tuple): A tuple of two integers (height, width) representing the dimensions of the image.

    Returns:
    list of tuples: List of normalized coordinates in the format [(x1, y1), (x2, y2), ...].
    """

    xyn = [(x / image_shape[1], y / image_shape[0]) for x, y in xy]
    return xyn


def box_iou_batch(boxes_true: np.ndarray, boxes_detection: np.ndarray) -> np.ndarray:
    """
    Compute Intersection over Union (IoU) of two sets of bounding boxes -
        `boxes_true` and `boxes_detection`. Both sets
        of boxes are expected to be in `(x_min, y_min, x_max, y_max)` format.

    Args:
        boxes_true (np.ndarray): 2D `np.ndarray` representing ground-truth boxes.
            `shape = (N, 4)` where `N` is number of true objects.
        boxes_detection (np.ndarray): 2D `np.ndarray` representing detection boxes.
            `shape = (M, 4)` where `M` is number of detected objects.

    Returns:
        np.ndarray: Pairwise IoU of boxes from `boxes_true` and `boxes_detection`.
            `shape = (N, M)` where `N` is number of true objects and
            `M` is number of detected objects.
    """

    def box_area(box):
        return (box[2] - box[0]) * (box[3] - box[1])

    area_true = box_area(boxes_true.T)
    area_detection = box_area(boxes_detection.T)

    top_left = np.maximum(boxes_true[:, None, :2], boxes_detection[:, :2])
    bottom_right = np.minimum(boxes_true[:, None, 2:], boxes_detection[:, 2:])

    area_inter = np.prod(np.clip(bottom_right - top_left, a_min=0, a_max=None), 2)
    return area_inter / (area_true[:, None] + area_detection - area_inter)

def non_max_suppression(
    predictions: np.ndarray, iou_threshold: float = 0.5
) -> np.ndarray:
    """
    Perform Non-Maximum Suppression (NMS) on object detection predictions.

    Args:
        predictions (np.ndarray): An array of object detection predictions in
            the format of `(x_min, y_min, x_max, y_max, score)`
            or `(x_min, y_min, x_max, y_max, score, class)`.
        iou_threshold (float, optional): The intersection-over-union threshold
            to use for non-maximum suppression.

    Returns:
        np.ndarray: A boolean array indicating which predictions to keep after n
            on-maximum suppression.

    Raises:
        AssertionError: If `iou_threshold` is not within the
            closed range from `0` to `1`.
    """
    assert 0 <= iou_threshold <= 1, (
        "Value of `iou_threshold` must be in the closed range from 0 to 1, "
        f"{iou_threshold} given."
    )
    rows, columns = predictions.shape

    # add column #5 - category filled with zeros for agnostic nms
    if columns == 5:
        predictions = np.c_[predictions, np.zeros(rows)]

    # sort predictions column #4 - score
    sort_index = np.flip(predictions[:, 4].argsort())
    predictions = predictions[sort_index]

    boxes = predictions[:, :4]
    categories = predictions[:, 5]
    ious = box_iou_batch(boxes, boxes)
    ious = ious - np.eye(rows)

    keep = np.ones(rows, dtype=bool)

    for index, (iou, category) in enumerate(zip(ious, categories)):
        if not keep[index]:
            continue

        # drop detections with iou > iou_threshold and
        # same category as current detections
        condition = (iou > iou_threshold) & (categories == category)
        keep = keep & ~condition

    return keep[sort_index.argsort()]

def polygon_to_mask(polygon: np.ndarray, resolution_wh: Tuple[int, int]) -> np.ndarray:
    """Generate a mask from a polygon.

    Args:
        polygon (np.ndarray): The polygon for which the mask should be generated,
            given as a list of vertices.
        resolution_wh (Tuple[int, int]): The width and height of the desired resolution.

    Returns:
        np.ndarray: The generated 2D mask, where the polygon is marked with
            `1`'s and the rest is filled with `0`'s.
    """
    width, height = resolution_wh
    mask = np.zeros((height, width))

    cv2.fillPoly(mask, [polygon], color=1)
    return mask

def mask_to_xyxy(masks: np.ndarray) -> np.ndarray:
    """
    Converts a 3D `np.array` of 2D bool masks into a 2D `np.array` of bounding boxes.

    Parameters:
        masks (np.ndarray): A 3D `np.array` of shape `(N, W, H)`
            containing 2D bool masks

    Returns:
        np.ndarray: A 2D `np.array` of shape `(N, 4)` containing the bounding boxes
            `(x_min, y_min, x_max, y_max)` for each mask
    """
    n = masks.shape[0]
    bboxes = np.zeros((n, 4), dtype=int)

    for i, mask in enumerate(masks):
        rows, cols = np.where(mask)

        if len(rows) > 0 and len(cols) > 0:
            x_min, x_max = np.min(cols), np.max(cols)
            y_min, y_max = np.min(rows), np.max(rows)
            bboxes[i, :] = [x_min, y_min, x_max, y_max]

    return bboxes


def mask_to_polygons(mask: np.ndarray) -> List[np.ndarray]:
    """
    Converts a binary mask to a list of polygons.

    Parameters:
        mask (np.ndarray): A binary mask represented as a 2D NumPy array of
            shape `(H, W)`, where H and W are the height and width of
            the mask, respectively.

    Returns:
        List[np.ndarray]: A list of polygons, where each polygon is represented by a
            NumPy array of shape `(N, 2)`, containing the `x`, `y` coordinates
            of the points. Polygons with fewer points than `MIN_POLYGON_POINT_COUNT = 3`
            are excluded from the output.
    """
    MIN_POLYGON_POINT_COUNT = 3
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    return [
        np.squeeze(contour, axis=1)
        for contour in contours
        if contour.shape[0] >= MIN_POLYGON_POINT_COUNT
    ]

def polygon_to_xyxy(polygon: np.ndarray) -> np.ndarray:
    """
    Converts a polygon represented by a NumPy array into a bounding box.

    Parameters:
        polygon (np.ndarray): A polygon represented by a NumPy array of shape `(N, 2)`,
            containing the `x`, `y` coordinates of the points.

    Returns:
        np.ndarray: A 1D NumPy array containing the bounding box
            `(x_min, y_min, x_max, y_max)` of the input polygon.
    """
    x_min, y_min = np.min(polygon, axis=0)
    x_max, y_max = np.max(polygon, axis=0)
    return np.array([x_min, y_min, x_max, y_max])

def filter_polygons_by_area(
    polygons: List[np.ndarray],
    min_area: Optional[float] = None,
    max_area: Optional[float] = None,
) -> List[np.ndarray]:
    """
    Filters a list of polygons based on their area.

    Parameters:
        polygons (List[np.ndarray]): A list of polygons, where each polygon is
            represented by a NumPy array of shape `(N, 2)`,
            containing the `x`, `y` coordinates of the points.
        min_area (Optional[float]): The minimum area threshold.
            Only polygons with an area greater than or equal to this value
            will be included in the output. If set to None,
            no minimum area constraint will be applied.
        max_area (Optional[float]): The maximum area threshold.
            Only polygons with an area less than or equal to this value
            will be included in the output. If set to None,
            no maximum area constraint will be applied.

    Returns:
        List[np.ndarray]: A new list of polygons containing only those with
            areas within the specified thresholds.
    """
    if min_area is None and max_area is None:
        return polygons
    ares = [cv2.contourArea(polygon) for polygon in polygons]
    return [
        polygon
        for polygon, area in zip(polygons, ares)
        if (min_area is None or area >= min_area)
        and (max_area is None or area <= max_area)
    ]


def move_boxes(xyxy: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """
    Parameters:
        xyxy (np.ndarray): An array of shape `(n, 4)` containing the bounding boxes
            coordinates in format `[x1, y1, x2, y2]`
        offset (np.array): An array of shape `(2,)` containing offset values in format
            is `[dx, dy]`.

    Returns:
        np.ndarray: Repositioned bounding boxes.

    Example:
        ```python
        >>> import numpy as np
        >>> import supervision as sv

        >>> boxes = np.array([[10, 10, 20, 20], [30, 30, 40, 40]])
        >>> offset = np.array([5, 5])
        >>> sv.move_boxes(boxes, offset)
        ... array([
        ...     [15, 15, 25, 25],
        ...     [35, 35, 45, 45]
        ... ])
        ```
    """
    return xyxy + np.hstack([offset, offset])

def scale_boxes(xyxy: np.ndarray, factor: float) -> np.ndarray:
    """
    Scale the dimensions of bounding boxes.

    Parameters:
        xyxy (np.ndarray): An array of shape `(n, 4)` containing the bounding boxes
            coordinates in format `[x1, y1, x2, y2]`
        factor (float): A float value representing the factor by which the box
            dimensions are scaled. A factor greater than 1 enlarges the boxes, while a
            factor less than 1 shrinks them.

    Returns:
        np.ndarray: Scaled bounding boxes.

    Example:
        ```python
        >>> import numpy as np
        >>> import supervision as sv

        >>> boxes = np.array([[10, 10, 20, 20], [30, 30, 40, 40]])
        >>> factor = 1.5
        >>> sv.scale_boxes(boxes, factor)
        ... array([
        ...     [ 7.5,  7.5, 22.5, 22.5],
        ...     [27.5, 27.5, 42.5, 42.5]
        ... ])
        ```
    """
    centers = (xyxy[:, :2] + xyxy[:, 2:]) / 2
    new_sizes = (xyxy[:, 2:] - xyxy[:, :2]) * factor
    return np.concatenate((centers - new_sizes / 2, centers + new_sizes / 2), axis=1)