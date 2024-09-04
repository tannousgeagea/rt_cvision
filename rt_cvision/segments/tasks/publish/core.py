import os
import uuid
import logging
import random
import numpy as np
from common_utils.services.kafka_manager import KafkaServiceManager
from configure.client import config_manager
parameters = config_manager.params.get('segmentation')

kafka_manager = KafkaServiceManager(
    config={'bootstrap.servers': f'{os.environ["KAFKA_HOST"]}:{os.environ["KAFKA_PORT"]}'}
)

kafka_manager.producer = kafka_manager.create_producer(kafka_manager.config)
topic_config = {
    'cleanup.policy': 'delete',  # Delete messages after they are consumed
    'compression.type': 'lz4',  # Use lz4 compression
    'delete.retention.ms': int(parameters.get('kafka_retention_ms')),  # Retain deleted messages for 10 seconds
    'file.delete.delay.ms': 2000,  # Delay file deletion by 2 seconds
}

topic_name = parameters.get('kafka_publish_topic')
kafka_manager.create_topic(
    topic_config=topic_config,
    topic_name=topic_name,
    replication_factor=1,
    num_partitions=1
)

def publish_to_kafka(params):
    success = False
    try:
        assert 'message' in params, f"Missing argument in publish_to_kafka: message"
        kafka_manager.publish_message(
            topic=topic_name,
            message=params.get('message'),
        )
        
        success = True
    except Exception as err:
        logging.error(f"Error while publishing segments messages to kafka: {err}")
        
    return success