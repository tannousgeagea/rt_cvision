import os
import cv2
import logging
import numpy as np
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
        self.object_size_est = ObjectSizeEst()
        self.kafka_publisher = KafkaPublisher(topic_name="cvision-dl-ops-core-waste-segments", config=self.config)
        self.tasks_runner = TaskRunner(tasks=self.tasks)

        logging.info("Parameters:")
        logging.info("---------------------")
        for key, value in self.config.items():
            logging.info(f"\t {key}: {value}")

        self.segmentation = Segmentation(config=self.config)
    
    def execute(self, cv_image, data:dict):
        try:
            logging.info(data)
            h0, w0, _ = cv_image.shape
            detections, filtered_regions, roi = self.segmentation.run(
                image=cv_image, 
                confidence_threshold=self.config.get("confidence_threshold", 0.25)
            )
            detections = self.object_size_est.execute(
                detections=detections, input_shape=cv_image.shape, correction_factor=0.0038
            )

            detections, unique_detection = self.segmentation.register(detections=detections)

            annotator = self.draw(cv_image, detections, roi, filtered_regions, data) 
            self.draw(annotator.im.data, unique_detection, None, [], data,color=(0, 0, 255))

            message = {
                "detections": detections.to_dict(),
                "roi": roi,
                "width": w0,
                "height": h0,
                "filterd_regions": filtered_regions,
                **data,
            }

            self.tasks_runner.run(
                tasks=["publish-to-kafka"],
                parameters=message,
            )
        except Exception as err:
            logging.error(f"Error while executing detections in segments: {err}")

    @property
    def tasks(self):
        return {
            "publish-to-kafka": self.kafka_publisher.publish
        }
    
    def draw(self, cv_image, detections, roi, filtered_regions, data, color:tuple = (0, 255, 0)):
        annotator = Annotator(im=cv_image.copy(), line_width=2)
        for i, box in enumerate(detections.xyxy):
            # txt_label = f"{detections.confidence[i]:.2f}"
            # txt_label += f"| {detections.object_length[i]:.2f}" if detections.object_length is not None else ''
            txt_label = f"| {detections.tracker_id[i]}" if detections.tracker_id is not None else ''
            annotator.box_label(
                box=box,
                label=txt_label,
                color=color
            )

        if roi:
            annotator.draw_mask(
                cnt=[np.array(roi)],
                alpha=0.2
            )

        for det in filtered_regions:
            xyxy = xyxyn2xyxy(det["det"], cv_image.shape)
            annotator.box_label(
                box=xyxy,
                label=f"{det['filter_type']}"
            )

        os.makedirs("/media/snapshots", exist_ok=True)
        cv2.imwrite(f"/media/snapshots/{os.path.basename(data['filename'])}", annotator.im.data)
        return annotator