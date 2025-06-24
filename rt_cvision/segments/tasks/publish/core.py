
import os
import logging
from typing import Dict
from common_utils.services.kafka_manager import KafkaServiceManager

class KafkaPublisher:
    def __init__(self, topic_name: str, config:dict):
        """
        KafkaPublisher wraps message production to a Kafka topic.

        Args:
            config_manager (ConfigManager): Instance of configuration manager.
            topic_key (str): Dot path in config_manager to locate the target topic name.
                             e.g., "segmentation.kafka_publish_topic"
        """
        self.topic_name = topic_name
        self.kafka_manager = KafkaServiceManager(
            config={
                'bootstrap.servers': f'{os.environ["KAFKA_HOST"]}:{os.environ["KAFKA_PORT"]}'
            }
        )
        self.kafka_manager.producer = self.kafka_manager.create_producer(self.kafka_manager.config)

        # Define topic config (can be customized per use case)
        topic_config = {
            'cleanup.policy': 'delete',
            'compression.type': 'lz4',
            'delete.retention.ms': int(config.get("kafka_retention_ms", 10000)),
            'file.delete.delay.ms': 2000,
        }

        self.kafka_manager.create_topic(
            topic_config=topic_config,
            topic_name=self.topic_name,
            replication_factor=1,
            num_partitions=1
        )

    
    def publish(self, message: Dict) -> bool:
        """
        Publishes a message to the configured Kafka topic.

        Args:
            message (Dict): The message payload to send.

        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            assert message is not None and isinstance(message, dict), "Message must be a non-empty dictionary"
            self.kafka_manager.publish_message(topic=self.topic_name, message=message)
            return True
        except Exception as err:
            logging.error(f"[KafkaPublisher] Failed to publish message: {err}")
            return False
