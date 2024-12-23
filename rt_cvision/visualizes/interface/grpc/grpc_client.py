import os
import grpc
import json
import sys
import uuid
import time
import logging
from visualizes.interface.grpc import visualizes_service_pb2
from visualizes.interface.grpc import visualizes_service_pb2_grpc

def run(messages):
    try:
        with grpc.insecure_channel(f'localhost:{os.environ.get("GRPC_VISUALIZES")}') as channel:
            start_time = time.time()
            stub = visualizes_service_pb2_grpc.ComputingUnitStub(channel)
            
            assert isinstance(messages, dict), f"message are expected to be dict, but got {type(messages)}"
            
            if not(len(messages)):
                return
            
            signal = {key: value for key, value in messages.items() if key!='cv_image'}
            
            data = json.dumps(signal)
            response = stub.ProcessData(visualizes_service_pb2.ProcessDataRequest(data=data))
            response_data = json.loads(response.result)
                        
            exectution_time = time.time() - start_time

            print(f"Execution Time: {int(exectution_time * 1000)} milliseconds!")
            print("Computing Service responded with updated data:", response_data)
            
    except Exception as err:
        logging.error(f"Error while reading and processing data in visualize module: {err}")
