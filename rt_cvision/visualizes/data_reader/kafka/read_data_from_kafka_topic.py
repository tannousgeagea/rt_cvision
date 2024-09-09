import os
import cv2
import json
import logging
import numpy as np
from common_utils.services.kafka_manager import KafkaServiceManager, KafkaError

kafka_config = {'bootstrap.servers': f'{os.environ["KAFKA_HOST"]}:{os.environ["KAFKA_PORT"]}'}
kafka_manager = KafkaServiceManager(
    config=kafka_config,
)

consumer_config = {
    **kafka_config,
    'auto.offset.reset': 'latest',
    'group.id': '0',
    }

kafka_manager.consumer = kafka_manager.create_consumer(conf=consumer_config)

def collect_messages(msg):
    info = {}
    try:
        info = msg.value().decode('utf-8')
        info = json.loads(info)
        print('Received message: {}'.format(info.keys()))
    except Exception as err:
        logging.error(f'Error trying collect message from kafka: {err}')
        
    return info
    
    
def get_data(callback=None):
    
    if callback is None:
        def callback(messages):
            print(
                'please define a custom callback to process the message!' + \
                    f'\ncurrent message contains {tuple(messages.keys())}'
            )
            
    def _callback(msg):
        messages = collect_messages(msg)
        callback(messages)
        
    kafka_manager.consume_message(topic='cvision-dl-ops-core-waste-segments', callback=_callback)
    
    
if __name__ == "__main__":
    get_data()