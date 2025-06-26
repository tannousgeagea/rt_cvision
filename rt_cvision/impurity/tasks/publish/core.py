import time
import rclpy
import logging
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from typing import Optional
logging.basicConfig(level=logging.INFO)

class ImagePublisher(Node):
    def __init__(self, config:Optional[dict], topic:Optional[str] = None, use_compressed: Optional[bool] = False,):
        super().__init__('impurity')
        self.bridge = CvBridge()
        self.config = config or {}
        self.topic = topic or self.config.get("ros_topic")
        if not self.topic:
            raise ValueError("ROS topic must be provided either via argument or config.")

        # Ensure use_compressed is boolean
        self.use_compressed = bool(
            use_compressed if use_compressed is not None else self.config.get("use_compressed", False)
        )
        self.publisher_ = self.create_publisher(
            CompressedImage if use_compressed else Image,
            topic,
            10
        )
        self.get_logger().info(
            f"[ImagePublisher] Initialized on topic '{topic}' (compressed={use_compressed})"
        )
        self._last_publish_time = None
        self._smoothed_fps = None
        self._ema_alpha = 0.2  # smoothing factor

    @property
    def rate(self) -> float:
        now = time.time()

        if self._last_publish_time is None:
            self._last_publish_time = now
            return 0.

        delta = now - self._last_publish_time
        self._last_publish_time = now

        fps = 1.0 / delta if delta > 0 else 0.0

        if self._smoothed_fps is None:
            self._smoothed_fps = fps
        else:
            self._smoothed_fps = (
                self._ema_alpha * fps +
                (1 - self._ema_alpha) * self._smoothed_fps
            )

        return self._smoothed_fps

    def publish_image(self, frame: np.ndarray):
        try:
            if frame is None:
                raise ValueError("Frame is None")

            header = Header()
            header.stamp = self.get_clock().now().to_msg()

            if self.use_compressed:
                success, encoded = cv2.imencode('.jpg', frame)
                if not success:
                    raise RuntimeError("Failed to encode image to JPEG.")

                msg = CompressedImage()
                msg.header = header
                msg.format = "jpeg"
                msg.data = encoded.tobytes()
            else:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header = header

            self.publisher_.publish(msg)
            fps = self.rate
            if fps:
                logging.info(f"✅ Published image to {self.topic} @ {fps:.2f} FPS (EMA)")
        except Exception as e:
            logging.error(f"❌ Failed to publish image: {e}")

def run(params: dict) -> bool:
    """
    Task to publish a ROS2 image.

    Expected keys in `params`:
    - 'cv_image': np.ndarray
    - 'image_publisher': an instance of ImagePublisher
    """
    try:
        frame = params.get("annotated_image")
        if frame is None:
            raise ValueError("cv_image is required and cannot be None.")

        image_publisher = params.get("image_publisher")
        if image_publisher is None:
            raise ValueError("image_publisher is required in params.")

        image_publisher.publish_image(frame)
        return True

    except Exception as err:
        logging.error(f"❌ Failed to publish image to ROS2: {err}")
        return False

