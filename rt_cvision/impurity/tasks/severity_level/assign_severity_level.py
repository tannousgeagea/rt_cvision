import logging
from typing import Union
from typing import Optional, Dict
from .mapping import (
    SEVERITY_LEVEL_MAP_BY_SIZE,
    SEVERITY_LEVEL_MAP_BY_CLASS
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