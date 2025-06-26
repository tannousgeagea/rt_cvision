# implementation of the gRPC server

import os
import sys
import cv2
import grpc
import json
import time
import logging
from concurrent import futures
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
from common_utils.services.redis_manager import RedisManager
from visualizes.interface.grpc import visualizes_service_pb2
from visualizes.interface.grpc import visualizes_service_pb2_grpc
from visualizes.main import Processor

redis_manager = RedisManager(
    host=os.environ['REDIS_HOST'],
    port=os.environ['REDIS_PORT'],
    db=os.environ['REDIS_DB'],
    password=os.environ['REDIS_PASSWORD'],
)

processor = Processor()
class ServiceImpl(visualizes_service_pb2_grpc.ComputingUnitServicer):
    def ProcessData(self, request, context):
        data = json.loads(request.data)
        print(f"Receiving Request: {data.keys()}")
        
        assert 'img_key' in data.keys(), f'key: img_key not found in data'
        img_key = data.get('img_key', 'None')
        status, retrieved_image = redis_manager.retrieve_image(key=img_key)
        
        assert status, f'Failed to retrieve image from Redis'
        assert not retrieved_image is None, f'Retrieved image is None'
        
        success = processor.run(
            cv_image=retrieved_image,
            data=data,
        )
        
        result = json.dumps(
            {
                'action': 'done',
                'suceess': str(success),
                'task_id': img_key,
                'datetime': data.get('datetime', None),
            }
        )
        
        return visualizes_service_pb2.ProcessDataResponse(result=result)
    
def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    visualizes_service_pb2_grpc.add_ComputingUnitServicer_to_server(ServiceImpl(), server)
    server.add_insecure_port(f'[::]:{os.environ.get("GRPC_VISUALIZES")}')
    server.start()
    server.wait_for_termination()
    
    
if __name__ == "__main__":
    serve()