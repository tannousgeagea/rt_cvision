import rclpy
import logging
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self, topic):
        super().__init__('visualizer')
        self.publisher_ = self.create_publisher(Image, topic, 10)  # Topic to publish to
        self.br = CvBridge()

    def publish_image(self, frame):
        if frame is not None:
            img_msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(img_msg)
            print('Published an image to image_topic')

rclpy.init(args=None)
image_publisher = ImagePublisher(topic='/sensor_processed/rgbmatrix_01/impurity_detection/live_mode')

def execute(params):
    try:
        assert 'cv_image' in params, f"Missing argument in publishing image to ros2: cv_image"
        assert params.get('cv_image') is not None, f"Received None Image while publishing to ROS2"
        
        image_publisher.publish_image(frame=params.get('cv_image'))  # Publish once
        # rclpy.spin(image_publisher)  # Keep the node alive

    except Exception as err:
        logging.error(f"Error while publishing image to ROS2 topic: {err}")


