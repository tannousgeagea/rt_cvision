#!/usr/bin/env python3

'''
 * Copyright (C) WasteAnt - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential
 * See accompanying/further LICENSES below or attached
 * Created by Tannous Geagea <tannous.geagea@wasteant.com>, March 2023
 * Edited by: Tannous Geagea <tannous.geagea@wasteant.com>
 *
'''

import os
import gc
import sys
import uuid
import psutil
import cv2
import time
import random
import logging
import numpy as np
from typing import Union, List
# os.environ['CUDA_VISIBLE_DEVICES'] = "-1"
from pathlib import Path
from datetime import datetime, timezone
from common_utils.annotate import Annotator
from common_utils.model import BaseModels
from wa_research_object_segmentation_classifier.src.common_utils.object_size.core import ObjectSize

base_dir = Path(__file__).resolve().parent.parent

def check_objects(objects:dict, keys:Union[str, List]):
    assert isinstance(objects, dict), f'objects are expected to be of type dict, but got {type(objects)}'
    for key in keys:
        assert key in objects.keys(), f'key {key} not found in objects'


def SEVERITY_LEVEL_MAP_BY_SIZE(o, thresholds:list=[0, 1., 1.5, 2.]):
    o = round(o * 100)
    for i, threshold in enumerate(thresholds):
        if o < threshold * 100:
            return (i - 1)
        
    return len(thresholds)

class Processor:

    def __init__(self, params):
        
        self.HOME = os.environ['HOME'] if 'HOME' in os.environ else '/home/wasteantadmin'
        self.CFG = params
        self.CFG['weights'] = base_dir / 'models' / self.CFG['weights']
        self.base_model = BaseModels(self.CFG)
        self.object_size = ObjectSize(self.CFG)


    def predict(self, image):
        start_time = time.time()
        results = self.base_model.classify_one(image, mode=self.CFG.get('mode', 'detect'))

        xyn = results.get('xyn')
        
        logging.info(f'Result: {len(xyn)}')
        check_objects(objects=results, 
                      keys=[
                          'xy',
                          'xyn',
                          'xyxy',
                          'xyxyn',
                          'confidence',
                          'class_id',
                      ])
        
        if self.CFG.get('mode') == "track":
            check_objects(objects=results, keys=['tracker_id'])

        end_time = time.time()
        logging.info('Inference Time:::::> %s s' % round(end_time - start_time, 4))

        return results

    def getData(self, image, objects):
        
        polygons = objects.get('xyn')
        
        if not polygons:
            objects['object_length'] = []
            objects['object_area'] = []
            return objects

        try:
            index, object_length, object_area = self.object_size.process(
                input_shape=image.shape,
                polygons=polygons,
                depth_map=None,
                eps=0.85,
                factor=self.CFG.get('object_size_factor', 0.006),
            )

            objects = {
                key: [value[i] for i in index]
                for key, value in objects.items()
                if isinstance(value, list)
            }
            
            
            objects['object_length'] = object_length
            objects['object_area'] = object_area            
            
        except Exception as err:
            logging.info('Unexpected Error while getting data: %s' %err)
            

        return objects
    
    def draw(self, image, instances, line_width=3, legend=None, colors=None, thresholds:list=None):
        """
        Annotates an image with bounding boxes and labels for each object described in the 'instances' dictionary.
        The annotations are customized based on the severity level of each object, with different colors representing
        different severity levels. The method utilizes an 'Annotator' class for drawing, which should be predefined
        and capable of drawing boxes and labels on images.

        Parameters:
        - image: The image to be annotated. This is expected to be a NumPy array or a similar object that represents
                an image data structure compatible with the annotator being used.
        - instances (dict): A dictionary containing the details of the objects to be annotated. Expected keys are:
            - 'boxes': A list of bounding boxes for each object in the format expected by the `xyxyn2xyxy` function,
                    presumably normalized coordinates (x_min, y_min, x_max, y_max, normalized to image size).
            - 'object_length': The length of each object, expected to be a list with a length value for each object.
            - 'severity_level': The severity level of each object, which influences the color of the box and label.
        - line_width (int, optional): The width of the lines used to draw the bounding boxes. Default is 3.

        Returns:
        - The annotated image, with bounding boxes and labels for each object based on their severity level and
        other attributes. The image is returned as a modified version of the input 'image', with annotations
        applied directly to it.

        Raises:
        - Logs a warning: If 'instances' is empty, indicating there are no objects to annotate on the image.
        - AssertionError: If any of the required keys ('boxes', 'object_length', 'severity_level') are missing in
                        'instances', or if the image is None, indicating that essential information is missing.
        - Logs an error: If an unexpected error occurs during the annotation process.

        Note:
        The function relies on several external elements and assumptions:
        - The `Annotator` class must be defined elsewhere and must provide an interface for drawing on images,
        including methods like `box_label`.
        - The `xyxyn2xyxy` function is assumed to be a utility for converting bounding box coordinates from one
        format to another and must be defined elsewhere.
        - The `severity_level_by_color` dictionary and the `severity_level_str` method must be defined within the
        class or globally to map severity levels to colors and convert severity levels to string representations,
        respectively. These mappings influence the visual appearance of the annotations.
        """        

        if not len(instances):
            logging.warning('No Instances have been Found ! - Skip')
            return image
        
        assert image is not None, f"Image is None"
        assert 'xyxy' in instances.keys(), f"bounding boxes of objects are not defined"
        
        annotator = Annotator(im=image.copy(), line_width=line_width)
        try:
            for i, box in enumerate(instances['xyxy']):
                # box = xyxyn2xyxy(box, image_shape=image.shape)
                label = ''  # default
                color = (0, 255, 0) # default
                if 'object_length' in instances.keys():
                    color = colors[SEVERITY_LEVEL_MAP_BY_SIZE(o=instances['object_length'][i], thresholds=thresholds)]
                    label = f"{round(instances['object_length'][i] * 100)} cm"
                
                annotator.box_label(
                    box=box,
                    label=label,
                    color=color,
                )
                
                
            if legend:
                annotator.legend(annotator.im.data,
                                 labels=legend,
                                 color=colors,
                                 line_width=line_width,
                            )
        except Exception as err:
            logging.error(f"Unexpected error while drawing alarm on image: %s" %err)
             
        return annotator.im.data