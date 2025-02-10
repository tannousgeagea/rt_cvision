import logging
from data_reader.interface.grpc import grpc_client
from data_reader.endpoints.files import read_data_from_files
# from data_reader.endpoints.ros import read_data_from_ros
from data_reader.endpoints.ros2 import read_data_from_ros2
from configure.client import ConfigManager
config_manager = ConfigManager()
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

mapping = {
    'files': read_data_from_files.read_data,
    'ros2': read_data_from_ros2.read_data,
}
params = {
    "src": config_manager.data_acquisition.src,
    "msg_type": config_manager.data_acquisition.msg_type,
    "ros_topic": config_manager.data_acquisition.ros_topic,
}

def main(mode="ros"):
    assert mode in mapping.keys(), f"mode is not supported: {mode}"
    print(f"Readind Data from {mode} ...")
    module = mapping.get(mode)
    if module:
        module(
            params=params,
            callback=grpc_client.run
            )
    
    
if __name__ == "__main__":
    main(mode=config_manager.data_acquisition.mode)