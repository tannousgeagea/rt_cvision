import logging
from typing import Union
from typing import Optional, Dict, List

def check(objects:Dict, threshold:Union[List, float]):
    try:
        if not 'object_length' in objects:
            return objects
        
        if isinstance(threshold, list):
            threshold = threshold[1] if len(threshold) > 1 else threshold[0]
            
        objects = {
            key: [
                value[i] for i in range(len(value)) if objects['object_length'][i] > threshold
            ]
            for key, value in objects.items()
            if isinstance(value, list)
        }
        
    except Exception as err:
        logging.error(f'Error in check objects size: {err}')
        
    return objects