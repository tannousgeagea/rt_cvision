
import logging
import numpy as np
from typing import Dict
from segments.utils.segments.core import Segmentation
from segments.utils.object_size.core import ObjectSizeEst
from configure.client import ServiceConfigClient
from common_utils.detection.convertor import xyxyn2xyxy
from common_utils.annotate.core import Annotator
from segments.tasks.publish.core import KafkaPublisher
from segments.tasks.core import TaskRunner

class Processor:
    def __init__(self) -> None:
        self.config_client = ServiceConfigClient(
            api_url="http://localhost:23085",
            service_id="segmentation"
        )

        self.config = self.config_client.load()
        self.map_tracker_id_2_object_uid = {}

        logging.info("Parameters:")
        logging.info("---------------------")
        for key, value in self.config.items():
            logging.info(f"\t {key}: {value}")

        self.object_size_est = ObjectSizeEst()
        self.kafka_publisher = KafkaPublisher(topic_name="cvision-dl-ops-core-waste-segments", config=self.config)
        self.tasks_runner = TaskRunner(tasks=self.tasks)
        self.segmentation = Segmentation(config=self.config)
    
    def run(self, cv_image, data:dict):
        try:
            logging.info(data)
            h0, w0, _ = cv_image.shape
            detections, filtered_regions, roi = self.segmentation.run(
                image=cv_image, 
                confidence_threshold=self.config.get("confidence_threshold", 0.25)
            )
            detections = self.object_size_est.execute(
                detections=detections, input_shape=cv_image.shape, correction_factor=self.config.get("correction_factor", 0.003)
            )

            detections = self.segmentation.classify(detections=detections)
            detections, unique_detections = self.segmentation.register(detections=detections)
            message = {
                "detections": detections.to_dict(),
                "unique_detections": unique_detections.to_dict(),
                "roi": roi,
                "width": w0,
                "height": h0,
                "filtered_regions": filtered_regions,
                "object-length-thresholds": self.config.get("object-length-thresholds"),
                **data,
            }

            self.tasks_runner.run(
                tasks=["publish-to-kafka"],
                parameters=message,
            )
        except Exception as err:
            logging.error(f"Error while executing detections in segments: {err}")

    @property
    def tasks(self) -> Dict:
        return {
            "publish-to-kafka": self.kafka_publisher.publish
        }