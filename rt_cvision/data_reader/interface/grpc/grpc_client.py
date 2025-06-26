import os
import grpc
import json
import sys
import uuid

import logging
from data_reader.interface.grpc import service_pb2
from data_reader.interface.grpc import service_pb2_grpc
from common_utils.services.redis_manager import RedisManager


redis_manager = RedisManager(
    host=os.environ['REDIS_HOST'],
    port=int(os.environ['REDIS_PORT']),
    db=int(os.environ['REDIS_DB']),
    password=os.environ['REDIS_PASSWORD'],
)

def run(payload):
    try:
        with grpc.insecure_channel(f'localhost:{os.environ.get("GRPC_DATA_READER")}') as channel:
            stub = service_pb2_grpc.ComputingUnitStub(channel)
            
            assert isinstance(payload, dict), f"payload are expected to be dict, but got {type(payload)}"
            assert 'cv_image' in payload.keys(), f"key: cv_image not found in payload"
            assert 'timestamp' in payload.keys(), f"key: timestamp not found in payload"
            
            if not(len(payload)):
                return
            
            cv_image = payload['cv_image']
            timestamp = payload['timestamp']
            signal = {key: value for key, value in payload.items() if key!='cv_image'}
            img_key = str(uuid.uuid4())
            status, img_key = redis_manager.handle_storage(cv_image, key=img_key, expire=int(os.environ.get('REDIS_EXPIRE', 5)))
            
            if not status:
                print('Failed to handle storage with Redis')
                return
            
            signal['img_key'] = img_key
            data = json.dumps(signal)
            response = stub.ProcessData(service_pb2.ProcessDataRequest(data=data))
            response_data = json.loads(response.result)
            
            print("Computing Service responded with updated data:", response_data)
    except Exception as err:
        print(err)

if __name__ == '__main__':
    
    payload = {'text': 'hello-world'}
    run(payload=payload)