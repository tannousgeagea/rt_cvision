import logging
import numpy as np
from typing import Union
from typing import Optional, Dict
from .mapping import (
    SEVERITY_LEVEL_MAP_BY_SIZE,
    SEVERITY_LEVEL_MAP_BY_CLASS,
    SEVERITY_LEVEL_MAP_BY_CLASS_vectorized,
    SEVERITY_LEVEL_MAP_BY_SIZE_vectorized
)

MAPPING = {
    'object_length': SEVERITY_LEVEL_MAP_BY_SIZE,
    'class_id': SEVERITY_LEVEL_MAP_BY_CLASS 
}

def SEVERITY_LEVEL_MAP(objects, key:str, thresholds:list):
    try:
        if not key in MAPPING.keys():
            logging.warning(
                f"⚠️  given key: {key} not found in SEVERITY LEVEL Mapping, expected keys: {list(MAPPING.keys())}" + \
                    "\nHowever key: object_length is assigned !"
                )
            
            key = 'object_length'
            
        assert key in objects, f'key: {key} not found in objects even though it is been used to assign severity level'
        
        mapping = MAPPING.get(key)
        objects["severity_level"] = [
            mapping(o, thresholds) for o in objects.get(key)
        ]
        
    except Exception as err:
        logging.error(f'Error in assigning severity level map: {err}')
        
    return objects

MAPPING = {
    'object_length': SEVERITY_LEVEL_MAP_BY_SIZE_vectorized,
    'class_id': SEVERITY_LEVEL_MAP_BY_CLASS_vectorized 
}

def SEVERITY_LEVEL_MAP_vectorized(objects, key: str, thresholds: list):
    """
    Assigns severity levels to objects using a vectorized mapping based on either object length or class id.
    
    Args:
        objects (dict): Dictionary containing object data.
        key (str): Key in the objects dictionary ('object_length' or 'class_id').
        thresholds (list): Thresholds to be used for mapping severity.
        
    Returns:
        dict: The updated objects dictionary with the "severity_level" key.
    """
    try:
        if key not in MAPPING:
            logging.warning(
                f"⚠️  given key: {key} not found in SEVERITY LEVEL Mapping, expected keys: {list(MAPPING.keys())}. " +
                "Defaulting to 'object_length'."
            )
            key = 'object_length'
            
        # Ensure the key exists in objects
        assert key in objects, f'Key: {key} not found in objects even though it is being used to assign severity level'
        
        mapping_func = MAPPING.get(key)
        values = np.asarray(objects.get(key))
        
        if key == 'object_length':
            severity = mapping_func(values, thresholds)
        elif key == 'class_id':
            severity = mapping_func(values, thresholds)
        else:
            severity = [mapping_func(o, thresholds) for o in values]
        
        objects["severity_level"] = severity.tolist() if isinstance(severity, np.ndarray) else severity
        
    except Exception as err:
        logging.error(f'Error in assigning severity level map: {err}')
        
    return objects
