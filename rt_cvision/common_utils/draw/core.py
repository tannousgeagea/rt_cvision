

import logging
import numpy as np
from datetime import datetime
from common_utils.detection.core import Detections
from common_utils.annotate.core import Annotator
from common_utils import DATETIME_FORMAT
from common_utils.annotate.color import Color
from common_utils.detection.convertor import xyxyn2xyxy, poly2xyxy
from common_utils.severity.utils import SEVERITY_LEVEL_MAP_BY_SIZE_vectorized
from common_utils.timezone_utils.timeloc  import get_location_and_timezone, convert_to_local_time

class BoxAnnotator:
    def __init__(self, config:dict):
        self.config = config
        self.show_class_label = self.config.get('show_class_label')
        self.show_attributes = self.config.get("show_attributes")
        self.show_context = self.config.get("show_context")
        self.show_object_size = self.config.get('show_object_size')
        self.show_roi = self.config.get('show_roi')
        self.show_tracker_id = self.config.get('show_tracker_id')
        self.show_filtered_regions = self.config.get('show_filterd_regions')
        self.show_legend = self.config.get('show_legend')
        self.show_timestamp = self.config.get('show_timestamp')
        self.line_width = self.config.get('line_width', None)
        self.timezone_str = get_location_and_timezone()

    def generate_random_rgb_vectorized(self, n=1):
        return np.random.randint(0, 256, size=(n, 3), dtype='uint8').tolist()

    def draw(
        self,
        cv_image,
        data:dict,
    ):

        detections = data.get("detections")
        if not detections:
            return cv_image
        
        detections = Detections.from_dict(
            results=detections
        )
        
        if detections.xyxyn is None:
            return cv_image
        
        colors = data.get('colors')
        if not colors:
            colors = self.generate_random_rgb_vectorized(n=len(detections))

        severity = data.get("severity")
        if not severity:
            if not detections.object_length is None:
                severity = SEVERITY_LEVEL_MAP_BY_SIZE_vectorized(o_values=detections.object_length, thresholds=data['thresholds'])
            else:
                severity = np.arange(len(detections))
            
        annotator = Annotator(im=cv_image.copy(), line_width=self.line_width)
        for detection_idx in range(len(detections)):
            label = ""
            if self.show_class_label and not detections.data is None and 'class_name' in detections.data:
                label += f"{detections.data['class_name'][detection_idx]} "
            if self.show_attributes and not detections.data is None and 'attributes' in detections.data and detections.data['attributes'][detection_idx]:
                label += f"{detections.data['attributes'][detection_idx]} "
            if self.show_context and not detections.data is None and 'context' in detections.data and detections.data['context'][detection_idx]:
                label += f"{detections.data['context'][detection_idx]['impurity_type']} "
            if self.show_object_size and not detections.object_length is None:
                label += f"{detections.object_length[detection_idx] * 100:.1f} cm "
            if self.show_tracker_id and not detections.tracker_id is None:
                label += f"{detections.tracker_id[detection_idx].astype(int)}"
            x1, y1, x2, y2 = detections.xyxy[detection_idx].astype(int)
            color = colors[severity[detection_idx]]
            
            annotator.box_label(
                box=(x1, y1, x2, y2),
                label=label,
                color=color
            )

        if self.show_filtered_regions and "filtered_regions" in data:
            for filtered_region in data['filtered_regions']:
                annotator.box_label(
                    box=xyxyn2xyxy(filtered_region['det'], image_shape=cv_image.shape),
                    label=f"{filtered_region['filter_type']}",
                    color=self.generate_random_rgb_vectorized(n=1)[0]
                )

        if self.show_roi and 'roi_pixels' in data and data['roi_pixels']:
            annotator.draw_enclosing_transparent_circle(
                box=poly2xyxy(data['roi_pixels']), 
                color=Color.GREEN.as_bgr(), 
                alpha=0.2)

        if self.show_legend and data.get('legend'):
            annotator.legend_v2(annotator.im.data,
                labels=data.get('legend'),
                color=colors,
                line_width=self.line_width,
            )
        
        if self.show_timestamp:
            annotator.add_legend(
                legend_text=convert_to_local_time(utc_time=datetime.now(), timezone_str=self.timezone_str).strftime(DATETIME_FORMAT),
            )

        return annotator.im.data
        
            



