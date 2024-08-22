import logging
from data_reader.interface.grpc import grpc_client
from data_reader.endpoints.from_files import read_data_from_files
from data_reader.endpoints.from_ros import read_data_from_ros

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

mapping = {
    'files': read_data_from_files.read_data,
    'ros': read_data_from_ros.read_data
}

params = {
    "topic": "/sensor_raw/rgbmatrix_01/image_raw/compressed",
    "msg_type": "compressed_image",
    "src": '/home/appuser/data/images',
}

def main(mode="ros"):
    assert mode in mapping.keys(), f"mode is not supported: {mode}"
    module = mapping.get(mode)
    if module:
        module(
            params=params,
            callback=grpc_client.run
            )
    
    
if __name__ == "__main__":
    main(mode='files')