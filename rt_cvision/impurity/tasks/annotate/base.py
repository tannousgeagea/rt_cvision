import logging
from typing import Dict
import numpy as np
from datetime import datetime
from common_utils.annotate.core import Annotator
from common_utils.detection.convertor import xyxyn2xyxy
from common_utils.timezone_utils.timeloc import get_location_and_timezone, convert_to_local_time

timezone_str = get_location_and_timezone()

DATETIME_FORMAT = "%Y-%m-%d %H:%M:%S"

def generate_random_rgb_vectorized(n=1):
    return np.random.randint(0, 256, size=(n, 3), dtype='uint8').tolist()

def draw(params):
    assert 'cv_image' in params.keys(), f'Missing argument in draw: cv_image'
    assert params.get('cv_image') is not None, f"Error in draw: Image is None"
    annotator = Annotator(im=params.get('cv_image'), line_width=params.get('line_width'))
    
    try:

        assert "objects" in params.keys(), f"missing argument in draw: objects"
        
        objects = params.get('objects')
        assert 'xyxy' in objects.keys(), f"bounding boxes of objects are not defined"
        assert 'object_length' in objects.keys(), f"object length of objects are not defined"
        assert 'severity_level' in objects.keys(), f"severity level of objects are not defined"
        
        colors = params.get('colors')
        if not colors:
            colors = generate_random_rgb_vectorized(n=max(objects.get('severity_level')) + 1)

        for i, box in enumerate(objects['xyxyn']):
            box = xyxyn2xyxy(box, image_shape=annotator.im.shape)
            color = colors[int(objects['severity_level'][i])]
            annotator.box_label(
                box=box,
                label=f"{round(objects['object_length'][i] * 100)} cm",
                color=color,
            )

        annotator.add_legend(
            legend_text=convert_to_local_time(datetime.now(), timezone_str=timezone_str).strftime(DATETIME_FORMAT) + f" | {params.get('entity_loc')}",
            font_scale=2,
            font_thickness=2
        )
        
        
        if params.get('legend'):
            annotator.legend(annotator.im.data,
                                labels=params.get('legend'),
                                color=colors,
                                line_width=params.get('line_width'),
                        )

        
    except Exception as err:
        logging.error(f"Error while annotating image in Impurity: {err}")
        
    return annotator.im.data

