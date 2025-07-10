import os
import cv2
import time
import rclpy
import logging
from rclpy.node import Node
from datetime import datetime
from sensor_msgs.msg import Image, CompressedImage
from common_utils.services.redis import redis_manager
from common_utils.time_utils import KeepTrackOfTime
from data_reader.utils.ros_utils import extract_data_from_topic

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


DATETIME_FORMAT = "%Y-%m-%d %H-%M-%S"
DEFAULT_ACQUISITION_RATE = 2 #fps
queue_size = 1
keep_track_of_time = KeepTrackOfTime()

print(f'ROS DISTRO: {os.environ.get("ROS_DISTRO")}')
print(os.environ)

class ImageSubscriber(Node):
    def __init__(self, topic, msg_type, callback=None):
        
        super().__init__('ros2_image_subscriber')
        print(f"Listening to Topic: {topic}")
        self.subscription = self.create_subscription(
            Image if msg_type=="image" else CompressedImage,
            topic,
            callback,
            10  # QoS (Quality of Service) profile depth
        )
        
        
def collect_messages(data):
    messages = {}
    try:
        for i, msg in enumerate(data):
            dt = datetime.now()
            messages = extract_data_from_topic(msg, messages)
            messages['datetime'] = dt.strftime(DATETIME_FORMAT)
            messages ['filename'] = f'{dt.strftime("%Y-%m-%d_%H-%M-%S")}.jpg'
            
    except Exception as err:
        logging.error(f"Unexpected error in extracting messages: {err}")

    return messages

def read_data(params, callback=None,  args=None):
    rclpy.init(args=args)
    # define  a default callback
    def default_callback(data):
        print(
                'Please define a custom callback!' + \
                    f'\n current payload contains {tuple(data.keys())}'
                )
          
    callback = callback if not callback is None else default_callback
    
    def _callback(*data):
        if redis_manager:
            raw = redis_manager.redis_client.get("ACQUISITION_RATE")
        else:
            raw = DEFAULT_ACQUISITION_RATE
        try:
            ACQUISITION_RATE = float(raw) if raw else DEFAULT_ACQUISITION_RATE
        except (ValueError, TypeError):
            ACQUISITION_RATE = DEFAULT_ACQUISITION_RATE

        print(f"Publishing at: {ACQUISITION_RATE} fps")
        if keep_track_of_time.check_if_time_less_than_diff(
            start=keep_track_of_time.what_is_the_time, 
            end=time.time(), 
            diff=(1 / (abs(ACQUISITION_RATE) + 1e-5)), #int((1 / (int(ACQUISITION_RATE) + 1)) if int(ACQUISITION_RATE) > 0 else 1)
            ):
            print("ignoring ... ... ")
            return
        
        keep_track_of_time.update_time(new=time.time())
        messages = collect_messages(data)
        callback(messages)  

    image_subscriber = ImageSubscriber(topic=params['ros_topic'], msg_type=params['msg_type'], callback=_callback)
    rclpy.spin(image_subscriber)
    
    
if __name__ == "__main__":
    read_data(
        params={
            "ros_topic": "/sensor_raw/rgbmatrix_02/image_raw",
        }
    )