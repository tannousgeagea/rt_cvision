import os
import uuid
import logging
import random
import numpy as np
from common_utils.services.kafka_manager import KafkaServiceManager

kafka_manager = KafkaServiceManager(
    config={'bootstrap.servers': 'localhost:9092'}
)

kafka_manager.producer = kafka_manager.create_producer(kafka_manager.config)
topic_config = {
    'cleanup.policy': 'delete',  # Delete messages after they are consumed
    'compression.type': 'lz4',  # Use lz4 compression
    'delete.retention.ms': 10000,  # Retain deleted messages for 10 seconds
    'file.delete.delay.ms': 2000,  # Delay file deletion by 2 seconds
}

topic_name = 'cvision-dl-ops-core-waste-segments'
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