import logging
from typing import Union
from typing import Optional, Dict, List

def check(objects:Dict, threshold:Union[List, float]):
    try:
        if not 'severity_level' in objects:
            return objects
        
        if isinstance(threshold, list):
            threshold = threshold[1] if len(threshold) > 1 else threshold[0]
            
        objects = {
            key: [
                value[i] for i in range(len(value)) if objects['severity_level'][i] > threshold
            ]
            for key, value in objects.items()
            if isinstance(value, list)
        }
        
    except Exception as err:
        logging.error(f'Error in checking objects problematic: {err}')
        
    return objects


