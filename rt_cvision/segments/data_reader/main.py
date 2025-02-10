import logging
from segments.interface.grpc import grpc_client
from segments.data_reader.kafka.read_data_from_kafka_topic import get_data
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

from configure.client import ConfigManager

config_manager = ConfigManager()

params  = {
    'kafka_topic': config_manager.segmentation.kafka_topic
}

def read_data():
    get_data(params=params, callback=grpc_client.run)
    
if __name__ == "__main__":
    read_data()