import os
import cv2
import time
import rclpy
import logging
from rclpy.node import Node
from datetime import datetime
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from common_utils.services.redis_manager import RedisManager
from common_utils.time_utils import KeepTrackOfTime
from data_reader.utils.ros_utils import extract_data_from_topic
import subprocess

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


DATETIME_FORMAT = "%Y-%m-%d %H-%M-%S"
DEFAULT_ACQUISITION_RATE = 2 #fps
queue_size = 1
keep_track_of_time = KeepTrackOfTime()


redis_manager = RedisManager(
    host=os.environ['REDIS_HOST'],
    port=os.environ['REDIS_PORT'],
    db=os.environ['REDIS_DB'],
    password=os.environ['REDIS_PASSWORD'],
)

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
        
        # self.monitor_topic_hz(topic=topic)
        
    def monitor_topic_hz(self, topic):
        try:
            # Run the `ros2 topic hz` command to monitor the topic frequency
            
            logging.info(f'monitoring {topic}....')
            process = subprocess.Popen(
                ["ros2", "topic", "hz", topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True  # Ensures output is in string format
            )
            
            output = process.stdout.readline()
            
            logging.info(output)
            # Continuously read the output from the subprocess
            while True:
                output = process.stdout.readline()
                if output:
                    print(f"[Topic Frequency]: {output.strip()}")
                # If the process ends, break the loop
                if process.poll() is not None:
                    break

        except Exception as e:
            logging.error(f"Error while monitoring topic frequency: {e}")
        
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
    print('hello')
    
    rclpy.init(args=args)
    
    # define  a default callback
    def default_callback(data):
        print(
                'Please define a custom callback!' + \
                    f'\n current payload contains {tuple(data.keys())}'
                )
        
        
    callback = callback if not callback is None else default_callback
    
    def _callback(*data):
        ACQUISITION_RATE = redis_manager.redis_client.get("ACQUISITION_RATE") or DEFAULT_ACQUISITION_RATE
        print(f"Publishing at: {ACQUISITION_RATE} fps")
        if keep_track_of_time.check_if_time_less_than_diff(
            start=keep_track_of_time.what_is_the_time, 
            end=time.time(), 
            diff=(1 / (int(ACQUISITION_RATE) + 1))
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