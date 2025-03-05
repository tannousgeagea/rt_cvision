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
from segments.interface.grpc import waste_segments_service_pb2
from segments.interface.grpc import waste_segments_service_pb2_grpc
from segments.tasks.publish.core import publish_to_kafka
from segments.tasks.segment import predictor
from segments.main import Processor

redis_manager = RedisManager(
    host=os.environ['REDIS_HOST'],
    port=os.environ['REDIS_PORT'],
    db=os.environ['REDIS_DB'],
    password=os.environ['REDIS_PASSWORD'],
)

processor = Processor()
class ServiceImpl(waste_segments_service_pb2_grpc.ComputingUnitServicer):
    def ProcessData(self, request, context):
        print(f"Receiving Request: {request.data}")
        data = json.loads(request.data)
        
        assert 'img_key' in data.keys(), f'key: img_key not found in data'
        img_key = data.get('img_key', 'None')
        status, retrieved_image = redis_manager.retrieve_image(key=img_key)
        
        assert status, f'Failed to retrieve image from Redis'
        assert not retrieved_image is None, f'Retrieved image is None'
        
        detections = predictor.predict(image=retrieved_image)
        # processor.execute(
        #     cv_image=retrieved_image, 
        #     detections=detections, 
        #     data=data,
        #     )
        
        result = json.dumps(
            {
                'action': 'done',
                **data,
            }
        )
        return waste_segments_service_pb2.ProcessDataResponse(result=result)
    
def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    waste_segments_service_pb2_grpc.add_ComputingUnitServicer_to_server(ServiceImpl(), server)
    server.add_insecure_port(f'[::]:{os.environ.get("GRPC_SEGMENTS")}')
    server.start()
    server.wait_for_termination()
    
    
if __name__ == "__main__":
    serve()