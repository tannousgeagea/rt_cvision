# implementation of the gRPC server
import os
import sys
import grpc
import json
import time
from concurrent import futures

from data_reader.interface.grpc import service_pb2
from data_reader.interface.grpc import service_pb2_grpc
from common_utils.services.kafka_manager import KafkaServiceManager

kafka_manager = KafkaServiceManager(
    config={'bootstrap.servers': f'{os.environ["KAFKA_HOST"]}:{os.environ["KAFKA_PORT"]}'}
)

kafka_manager.producer = kafka_manager.create_producer(kafka_manager.config)
topic_config = {
    'cleanup.policy': 'delete',  # Delete messages after they are consumed
    'compression.type': 'lz4',  # Use lz4 compression
    'delete.retention.ms': 10000,  # Retain deleted messages for 10 seconds
    'file.delete.delay.ms': 2000,  # Delay file deletion by 2 seconds
}

topic_name = 'cvision-dl-ops-core-data-acquisition'
kafka_manager.create_topic(
    topic_config=topic_config,
    topic_name=topic_name,
    replication_factor=1,
    num_partitions=1
)

class ServiceImpl(service_pb2_grpc.ComputingUnitServicer):
    def ProcessData(self, request, context):
        print(f"Receiving Request: {request.data}")
        data = json.loads(request.data)
        
        kafka_manager.publish_message(topic=topic_name, message=data)
        result = json.dumps(data)
        return service_pb2.ProcessDataResponse(result=result)
    
def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    service_pb2_grpc.add_ComputingUnitServicer_to_server(ServiceImpl(), server)
    server.add_insecure_port(f'[::]:{os.environ.get("GRPC_DATA_READER")}')
    server.start()
    server.wait_for_termination()
    
    
if __name__=="__main__":
    serve()
    
    