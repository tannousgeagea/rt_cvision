def _get_center(bbox):
    """
    Compute the center of a bounding box.
    - bbox: [x_min, y_min, x_max, y_max]

    Returns:
        (center_x, center_y)
    """
    center_x = (bbox[0] + bbox[2]) / 2
    center_y = (bbox[1] + bbox[3]) / 2
    return (center_x, center_y)

def _is_within(point, roi):
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