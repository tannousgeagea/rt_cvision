
import os
import cv2
import uuid
import rclpy
import logging
import numpy as np
from typing import Optional, Dict, cast
from common_utils.model.base import BaseModels
from common_utils.detection.core import Detections
from configure.client import ServiceConfigClient
from impurity.utils.detect.core import DetectionModel
from impurity.tasks.core import TaskRunner
from common_utils.annotate.color import Color
from common_utils.draw.core import BoxAnnotator
from impurity.tasks.publish.core import ImagePublisher, run as ros2_run
from common_utils.severity.utils import SEVERITY_LEVEL_MAP_BY_CLASS_vectorized
from impurity.utils.database.core import DatabaseManager

class Processor:
    def __init__(self) -> None:
        self.map_tracker_id_2_object_uid = {}
        self.config_client = ServiceConfigClient(
            api_url="http://localhost:23085",
            service_id="impurity"
        )

        self.config = self.config_client.load()
        self.config["tenant"] = self.config_client.get_tenant_context()
        logging.info("Parameters:")
        logging.info("---------------------")
        for key, value in self.config.items():
            logging.info(f"\t {key}: {value}")

        self.db_manager = DatabaseManager(self.config)
        self.tasks_runner = TaskRunner(self.tasks)

        rclpy.init()
        self.config["show_legend"] = True
        self.config["show_timestamp"] = True
        self.config["show_object_size"] = False
        self.config['show_class_label'] = True
        self.config['show_attributes'] = False
        self.config['show_context'] = True
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
                    colors.append(Color.from_hex(threshold['color']).as_bgr())
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
            detections = self.detection_models.check_duplicate(detections)
            detections, pdetections = self.detection_models.classify(detections=detections)
            severity = SEVERITY_LEVEL_MAP_BY_CLASS_vectorized(classes=detections.class_id, thresholds=[1, 2, 3])
            logging.info(f"[Impurity] Severity {severity}")
            logging.info(f"[Impurity] Classes: {detections.data['class_name']}")
            logging.info(f"[Impurity] Attributes: {detections.data.get('attributes')}")
            logging.info(f"[Impurity] Context: {detections.data['context']}")
            logging.info(f"[Impurity] {len(detections)} detections vs {len(pdetections)} problematic detections")

            message = {
                "cv_image": cv_image.copy(),
                "thresholds": thresholds,
                "colors": colors,
                "legend": labels,
                "severity": severity.tolist(),
                "detections": detections.to_dict(),
                "pdetections": pdetections.to_dict(),
                **self.config,
                "image_publisher": self.ipublisher,
            }
         
            image = self.box_annotator.draw(cv_image=cv_image, data=message)
            message["annotated_image"] = image

            self.tasks_runner.run(
                tasks=[
                    "save-detections", 
                    "publish-ros2"
                ],
                parameters=message,
            )
            
        except Exception as err:
            logging.error(f"[Impurity] Error while executing detections in impurity: {err}")

    @property
    def tasks(self) -> Dict:
        return {
            "save-detections": self.db_manager.save,
            "publish-ros2": ros2_run
        }