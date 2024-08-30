import os
import time
import logging
from datetime import datetime
from data_reader.utils.ros_utils import extract_data_from_topic
from common_utils.services.ros_manager import ROSManager
from common_utils.services.redis_manager import RedisManager
from common_utils.time_utils import KeepTrackOfTime

DATETIME_FORMAT = "%Y-%m-%d %H-%M-%S"
DEFAULT_ACQUISITION_RATE = 3 #fps
queue_size = 1
keep_track_of_time = KeepTrackOfTime()


redis_manager = RedisManager(
    host=os.environ['REDIS_HOST'],
    port=os.environ['REDIS_PORT'],
    db=os.environ['REDIS_DB'],
    password=os.environ['REDIS_PASSWORD'],
)

def collect_messages(data):
    messages = {}
    try:
        for i, msg in enumerate(data):
            dt = datetime.now()
            messages = extract_data_from_topic(msg, messages)
            messages['datetime'] = dt.strftime(DATETIME_FORMAT)
            messages ['filename'] = dt.strftime("%Y-%m-%d_%H-%M-%S")
            
    except Exception as err:
        logging.error(f"Unexpected error in extracting messages: {err}")

    return messages

def read_data(params, callback=None):
    # define  a default callback
    def default_callback(*data):
        print(f'running default callback: {type(data)}')
        
    callback = callback if not callback is None else default_callback
        
    def _callback(*data):
        ACQUISITION_RATE = redis_manager.redis_client.get("ACQUISITION_RATE") or DEFAULT_ACQUISITION_RATE
        print(f"Publishing at: {ACQUISITION_RATE} fps")
        if keep_track_of_time.check_if_time_less_than_diff(
            start=keep_track_of_time.what_is_the_time, 
            end=time.time(), 
            diff=(1 / int(ACQUISITION_RATE))
            ):
            print("ignoring ... ... ")
            return
        
        keep_track_of_time.update_time(new=time.time())
        messages = collect_messages(data)
        callback(messages)    
    
    try:
        
        assert "topic" in params.keys(), f"key: topic not found in params"
        assert "msg_type" in params.keys(), f"key: msg_type not found in params"
        
        ros_manager = ROSManager(
            topics=params["topic"],
            msg_type=params["msg_type"],
            callback=_callback,
            processor_node_name='cvision_dl_ops_core_data_acquisition_ros_subscriber'
        )
        
        
        print("startig ros process ...")
        ros_manager.init_node()
        ros_manager.listener_on(queue_size=queue_size)

    except Exception as err:
        logging.error(f'Error in getting data from ROS: {err}')

    
    