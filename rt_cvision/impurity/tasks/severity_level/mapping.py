
import numpy as np

def SEVERITY_LEVEL_MAP_BY_SIZE(o, thresholds:list=[0, 1., 1.5, 2.]):
    o = round(o * 100)
    for i, threshold in enumerate(thresholds):
        if o < threshold * 100:
            return (i - 1)
        
    return len(thresholds) - 1

def SEVERITY_LEVEL_MAP_BY_CLASS(c, thresholds:list=[0, 1, 2, 3]):
    if c > len(thresholds):
        return len(thresholds) - 1
    
    return thresholds[c]

def SEVERITY_LEVEL_MAP_BY_SIZE_vectorized(o_values, thresholds=[0, 1., 1.5, 2.]):
    """
    Vectorized mapping for object length.
    
    Args:
        o_values (np.ndarray): Array of object lengths.
        thresholds (list): Thresholds (in metric units) to define severity levels.
    
    Returns:
        np.ndarray: Severity levels for each object.
    """
    o_values = np.round(o_values * 100)
    thresholds_arr = np.array(thresholds) * 100
    indices = np.searchsorted(thresholds_arr, o_values, side='right') - 1
    indices = np.clip(indices, 0, len(thresholds) - 1)
    return indices

def SEVERITY_LEVEL_MAP_BY_CLASS_vectorized(classes, thresholds=[0, 1, 2, 3]):
    """
    Vectorized mapping for class-based severity.
    
    Args:
        classes (np.ndarray or list): Array of class ids from the model.
        thresholds (list): Predefined severity levels corresponding to class ids.
        
    Returns:
        np.ndarray: Severity levels mapped from class ids.
    """
    classes = np.array(classes)
    thresholds_arr = np.array(thresholds)
    severity = np.where(classes >= len(thresholds), thresholds_arr[-1], thresholds_arr[classes])
    return severity
