import logging
from data_reader.interface.grpc import grpc_client
from data_reader.endpoints.files import read_data_from_files
from data_reader.endpoints.ros2 import read_data_from_ros2
from configure.client import ServiceConfigClient
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

mapping = {
    'files': read_data_from_files.read_data,
    'ros2': read_data_from_ros2.read_data,
}

config_client = ServiceConfigClient(
    api_url="http://localhost:23085",
    service_id="data_acquisition",
)

params = {
    "src": config_client.get("files"),
    "msg_type": config_client.get("msg_type"),
    "ros_topic": config_client.get("ros_topic"),
}

def main(mode):
    assert mode in mapping.keys(), f"mode is not supported: {mode}"
    logging.info(f"Readind Data from {mode} ...")

    config = config_client.load()
    logging.info("Parameters:")
    logging.info("---------------------")
    for key, value in config.items():
        logging.info(f"\t {key}: {value}")

    module = mapping.get(mode)
    if module:
        module(
            params=params,
            callback=grpc_client.run
            )
    
    
if __name__ == "__main__":
    main(mode=config_client.get("acquisition-mode", "ros2"))