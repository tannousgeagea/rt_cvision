import logging
from visualizes.interface.grpc import grpc_client
from visualizes.data_reader.kafka.read_data_from_kafka_topic import get_data
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def read_data():
    get_data(callback=grpc_client.run)
    
if __name__ == "__main__":
    read_data()