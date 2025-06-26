
import os
import cv2
import uuid
import rclpy
import logging
import threading
import numpy as np
from typing import Optional, Dict, cast
from datetime import datetime, timezone
from common_utils.model.base import BaseModels
from impurity.utils.objects.core import ObjectManager
from common_utils.detection.core import Detections
from configure.client import ServiceConfigClient
from impurity.utils.detect.core import DetectionModel
from impurity.tasks.core import TaskRunner
from common_utils.annotate.color import Color
from visualizes.utils.draw.core import BoxAnnotator
from impurity.tasks.publish.core import ImagePublisher, run as ros2_run
from common_utils.detection.assign import enrich_segments_with_detections

class Processor:
    def __init__(self) -> None:
        self.map_tracker_id_2_object_uid = {}
        self.config_client = ServiceConfigClient(
            api_url="http://localhost:23085",
            service_id="impurity"
        )

        self.config = self.config_client.load()
        logging.info("Parameters:")
        logging.info("---------------------")
        for key, value in self.config.items():
            logging.info(f"\t {key}: {value}")

        self.tasks_runner = TaskRunner(self.tasks)

        rclpy.init()
        self.config["show_legend"] = True
        self.config["show_timestamp"] = True
        self.config["show_object_size"] = True
        self.config['show_class_label'] = True
        self.box_annotator = BoxAnnotator(config=self.config)
        self.ipublisher = ImagePublisher(config=self.config, topic="/rgb/left/impurity/enriched")
        self.detection_models = DetectionModel(self.config)

    def execute(self, cv_image:np.ndarray, data:Dict, classes=None):
        try:
            object_length_threshold = data.get("object-length-thresholds")
            if object_length_threshold:
                labels, colors, thresholds = [], [], []
                for threshold in object_length_threshold:
                    if threshold["max"]:
                        labels.append(f"{threshold['min']} - {threshold['max']}")
                    else:
                        labels.append(f" > {threshold['min']}")
                    colors.append(Color.from_hex(threshold['color']).as_rgb())
                    thresholds.append(threshold['min'])

            unique_segments = Detections.from_dict(results=data["unique_detections"])
            logging.info(f"[Impurity] Segments: {len(unique_segments)}")
            indices = np.where(unique_segments.object_length >= thresholds[1])[0]
            unique_segments = cast(Detections, unique_segments[indices])

            logging.info(f"[Impurity] Potential Segments: {len(unique_segments)}")
            detections = self.detection_models.run(
                segments=unique_segments, image=cv_image, confidence_threshold=self.config.get("confidence_threshold", 0.25)
            )
            logging.info(f"[Impurity] Enriched Segments: {len(detections)}")

            message = {
                "cv_image": cv_image.copy(),
                "thresholds": thresholds,
                "colors": colors,
                "legend": labels,
                "detections": detections.to_dict(),
                **self.config,
                "image_publisher": self.ipublisher,
            }
            
            image = self.box_annotator.draw(cv_image=cv_image, data=message)
            message["annotated_image"] = image

            self.tasks_runner.run(
                tasks=["publish-ros2"],
                parameters=message,
            )

            # filename = f"{data['filename']}"
            # cv2.imwrite(f"/media/snapshots/{filename}", image)
            # if not len(segments):
            #     print('No Qualified Objects')
            #     return
            
            # detections = model.classify_one(cv_image, conf=config_manager.impurity.conf, is_json=False)
            # if classes:
            #     detections = detections[np.isin(detections.class_id, classes)]
            
            # if not len(detections):
            #     print('No Detection ! 🕵️‍♂️🔍❌ ')
            #     return 
            
            # detections = detections.with_nms()
            # object_manager = ObjectManager(
            #     segments=segments, detections=detections
            # )
            
            # detections = object_manager.is_problematic()
            # if detections is None:
            #     return

            # object_manager.severtiy_level(mapping_key=mapping_key, mapping_threshold=mapping_threshold)
            # object_manager.save(cv_image=cv_image)
            # return detections
            
        except Exception as err:
            logging.error(f"[Impurity] Error while executing detections in impurity: {err}")

    @property
    def tasks(self) -> Dict:
        return {
            "publish-ros2": ros2_run
        }