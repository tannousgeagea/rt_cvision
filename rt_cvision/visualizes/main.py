
import rclpy
import logging
from typing import Dict
from configure.client import ServiceConfigClient
from common_utils.annotate.color import Color
from visualizes.utils.draw.core import BoxAnnotator
from visualizes.tasks.publish.core import ImagePublisher, run as run_ros2
from visualizes.tasks.core import TaskRunner

class Processor:
    def __init__(self) -> None:
        self.config_client = ServiceConfigClient(
            api_url="http://localhost:23085",
            service_id="visualizer"
        )

        self.config = self.config_client.load()
        self.box_annotator = BoxAnnotator(config=self.config)

        logging.info("Parameters:")
        logging.info("---------------------")
        for key, value in self.config.items():
            logging.info(f"\t {key}: {value}")

        rclpy.init()
        self.ipublisher = ImagePublisher(config=self.config, topic=self.config.get("ros_topic"))
        self.task_runner = TaskRunner(tasks=self.tasks)


    def run(self, cv_image, data):
        success = False
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

            message = {
                "cv_image": cv_image.copy(),
                "thresholds": thresholds,
                "colors": colors,
                "legend": labels,
                **data,
                **self.config,
                "image_publisher": self.ipublisher,
            }
            
            image = self.box_annotator.draw(cv_image=cv_image, data=message)
            message['annotated_image'] = image
            self.task_runner.run(
                tasks=["publish-ros2"],
                parameters=message,
            )
            
        except Exception as err:
            logging.error(f"Error in visulaze: {err}")
        
        return success
    
    @property
    def tasks(self) -> Dict:
        return {
            "publish-ros2": run_ros2
        }