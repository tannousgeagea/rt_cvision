import os
import grpc
import json
import sys
import uuid
import time
import logging
from common_utils.services.kafka_manager import KafkaServiceManager
from segments.interface.grpc import waste_segments_service_pb2
from segments.interface.grpc import waste_segments_service_pb2_grpc
from common_utils.services.redis_manager import RedisManager
from segments.tasks.database import register

redis_manager = RedisManager(
    host=os.environ['REDIS_HOST'],
    port=os.environ['REDIS_PORT'],
    db=os.environ['REDIS_DB'],
    password=os.environ['REDIS_PASSWORD'],
)


def run(messages):
    try:
        with grpc.insecure_channel(f'localhost:{os.environ.get("GRPC_SEGMENTS")}') as channel:
            start_time = time.time()
            stub = waste_segments_service_pb2_grpc.ComputingUnitStub(channel)
            
            assert isinstance(messages, dict), f"message are expected to be dict, but got {type(messages)}"
            
            if not(len(messages)):
                return
            
            signal = {key: value for key, value in messages.items() if key!='cv_image'}
            
            data = json.dumps(signal)
            response = stub.ProcessData(waste_segments_service_pb2.ProcessDataRequest(data=data))
            response_data = json.loads(response.result)
            
            exectution_time = time.time() - start_time
            redis_manager.redis_client.set("ACQUISITION_RATE", int(1 / exectution_time))
            
            print(f"Execution Time: {int(exectution_time * 1000)} milliseconds!")
            print("Computing Service responded with updated data:", response_data)
            
    except Exception as err:
        logging.error(f"Error while reading and processing data in segmentation module: {err}")

if __name__ == '__main__':
    messages = {'text': 'hello-world'}
    run(messages=messages)