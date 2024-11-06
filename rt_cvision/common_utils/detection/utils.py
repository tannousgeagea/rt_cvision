
import numpy as np
from typing import Dict, List, Optional, Tuple, Union

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


def box_non_max_suppression(
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

def get_data_item(
    data: Dict[str, Union[np.ndarray, List]],
    index: Union[int, slice, List[int], np.ndarray],
) -> Dict[str, Union[np.ndarray, List]]:
    """
    Retrieve a subset of the data dictionary based on the given index.

    Args:
        data: The data dictionary of the Detections object.
        index: The index or indices specifying the subset to retrieve.

    Returns:
        A subset of the data dictionary corresponding to the specified index.
    """
    subset_data = {}
    for key, value in data.items():
        if isinstance(value, np.ndarray):
            subset_data[key] = value[index]
        elif isinstance(value, list):
            if isinstance(index, slice):
                subset_data[key] = value[index]
            elif isinstance(index, list):
                subset_data[key] = [value[i] for i in index]
            elif isinstance(index, np.ndarray):
                if index.dtype == bool:
                    subset_data[key] = [
                        value[i] for i, index_value in enumerate(index) if index_value
                    ]
                else:
                    subset_data[key] = [value[i] for i in index]
            elif isinstance(index, int):
                subset_data[key] = [value[index]]
            else:
                raise TypeError(f"Unsupported index type: {type(index)}")
        else:
            raise TypeError(f"Unsupported data type for key '{key}': {type(value)}")

    return subset_data

def get_data_from_list(
    data: List,
    index: Union[int, slice, List[int], np.ndarray],
):
    assert isinstance(data, List), f'data must be of type list, but got {type(data)}'
    if isinstance(index, int):
        index = [index]
        
    if isinstance(index, slice):
        index = range(index.start, index.stop, index.step if index.step is not None else 1)

    subset_data = [data[i] for i in index]
    
    return subset_data

def adjust_to_original(coords, offset, crop_size, original_size, mode='xyxy'):
    """
    Adjusts coordinates to the original image space for various formats.
    
    Parameters:
    - coords (np.ndarray): Coordinates to adjust (either `xyxy`, `xyxyn`, `xy`, or `xyn`).
    - offset (tuple): The top-left corner (x_min, y_min) of the cropped region in the original image.
    - crop_size (tuple): Width and height of the cropped region (w, h).
    - original_size (tuple): Width and height of the original image (W, H).
    - mode (str): Mode of coordinates, one of 'xyxy', 'xyxyn', 'xy', or 'xyn'.
    
    Returns:
    - adjusted_coords (np.ndarray): Coordinates adjusted to the original image space.
    """
    x_min, y_min = offset
    crop_h, crop_w = crop_size
    original_h, original_w = original_size
    if mode == 'xyxy':
        adjusted_coords = coords + np.array([x_min, y_min, x_min, y_min])
    
    elif mode == 'xyxyn':
        adjusted_coords = coords.copy()
        adjusted_coords[0] = (coords[0] * crop_w + x_min) / original_w  # x1 normalized
        adjusted_coords[1] = (coords[1] * crop_h + y_min) / original_h  # y1 normalized
        adjusted_coords[2] = (coords[2] * crop_w + x_min) / original_w  # x2 normalized
        adjusted_coords[3] = (coords[3] * crop_h + y_min) / original_h  # y2 normalized
    
    
    elif mode == 'xy':
        adjusted_coords = coords + np.array([x_min, y_min])
    
    elif mode == 'xyn':
        adjusted_coords = coords.copy()
        adjusted_coords[:, 0] = (coords[:, 0] * crop_w + x_min) / original_w  # x normalized
        adjusted_coords[:, 1] = (coords[:, 1] * crop_h + y_min) / original_h  # y normalized
    
    else:
        raise ValueError("Mode should be one of 'xyxy', 'xyxyn', 'xy', or 'xyn'")
    
    return adjusted_coords