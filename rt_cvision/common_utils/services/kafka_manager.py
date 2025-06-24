
import json
import time
import logging
import numpy as np
from confluent_kafka import Producer
from confluent_kafka import Consumer, KafkaError
from confluent_kafka.admin import AdminClient, NewTopic

class KafkaServiceManager:
    def __init__(self, config=None, offset=None):
        self.config = config
        self.admin_client = AdminClient(self.config)
        self.producer = None
        self.consumer = None
        # self.producer = self.create_producer(self.producer_config)
        # self.consumer = self.create_consumer(self.consumer_config, offset=offset)
        
    def create_producer(self, conf):
        """Initialize and return a Kafka producer based on the provided config"""
        return Producer(**conf)
        
    def create_consumer(self, conf, offset=None):
        """Initialize and return a Kafka consumer based on the provided config"""
        assert 'auto.offset.reset' in conf, f"offset rules is not defined. please provide offset" + \
                "you can define offset either:\n" + \
                "\t1. by passing to config file -> conf['auto.offset.reset] = earliest for example\n" + \
                "\t2. by passing to the consumer directly and it will be overwritten: KafkaServiceManager.create_consumer(conf, offset='earliest)"
                
        conf['auto.offset.reset'] = offset if offset is not None else conf['auto.offset.reset']
        return Consumer(**conf)
    
    def publish_message(self, topic, message):
        """
        Publish a message to the specified Kafka topic. This method supports messages in
        string or dictionary format. Dictionaries are automatically converted to JSON strings
        before being published. It utilizes an asynchronous callback to confirm message delivery.

        Parameters:
        - topic (str): The Kafka topic to which the message will be published.
        - message (str or dict): The message to be published. If the message is a dictionary,
        it will be converted to a JSON string.

        Raises:
        - KafkaError: If there is an error in delivering the message to the specified topic.
        - ValueError: If the message is neither a string nor a dictionary.

        Example usage:
        >>> kafka_manager = KafkaServiceManager(producer_config)
        >>> kafka_manager.publish_message('your_topic', 'Hello, Kafka!')
        >>> kafka_manager.publish_message('your_topic', {'key': 'value'})
        
        Note: This method prints a confirmation message to the console upon successful delivery,
        including the topic, partition, and offset of the published message. In case of an error,
        a KafkaError is raised with details of the failure.
        """
        def acked(err, msg):
            if err is not None:
                raise KafkaError(f'Failed to deliver message: {err}')
            else:
                print(f"Message produced: {msg.topic()} {msg.partition()} {msg.offset()}")

        assert not self.producer is None, f"producer is not initialized yet. initialize producer with self.kafka_service_manager.create_producer(conf)"

        if isinstance(message, str):
            info = str(message)
        elif isinstance(message, dict):
            info = json.dumps(message)
        else:
            raise ValueError(f'Undefined message type:{message}. Expected [str, dict] but got {type(message)}')
        
        self.producer.produce(topic, info.encode('utf-8'), callback=acked)
        self.producer.flush()

    def consume_message(self, topic, callback=None):
        """
        Consumes messages from a specified Kafka topic and processes each message using a callback function. If no callback is provided,
        a default callback is used that prints the message value. The function subscribes to the given topic and polls for messages
        continuously until interrupted.

        Parameters:
        - topic (str): The name of the Kafka topic from which to consume messages.
        - callback (function, optional): A function to process each message received from the topic. The function should take a single
        parameter, which is a message object. If no callback is provided, a default callback that prints the message's value is used.

        Raises:
        - AssertionError: If the consumer has not been initialized before calling this method. Ensure that the consumer is initialized
        with the necessary configuration, including 'auto.offset.reset'.
        - ValueError: If an error is encountered while consuming messages, except for the end of partition event, which is silently ignored.

        Note:
        - The function blocks and continuously polls for messages until manually interrupted (e.g., via KeyboardInterrupt).
        - It is crucial to properly manage and close the consumer to avoid resource leaks. This function handles consumer closure upon
        interruption or after encountering an error.

        Example usage:
        >>> def process_message(msg):
        ...     print(f"Received message: {msg.value().decode('utf-8')}")
        >>> kafka_manager = KafkaServiceManager(producer_config={}, consumer_config=consumer_config)
        >>> kafka_manager.consume_message('your_topic', process_message)
        """
        
        # def default_callback(msg):
        #     print(f'this is a default callback - define your own callback to work on the extracted message {msg.value().decode('utf-8')}')
        
        # callback = callback if callback is not None else default_callback
        assert not self.consumer is None, f"consumer is not initialized yet. initialize consumer with self.kafka_service_manager.create_consumer(conf)." + \
            "make sure to define conf['auto.offset.reset']"
            
        
        self.consumer.subscribe([topic])
        try:
            while True:
                msg = self.consumer.poll(timeout=1.0)
                if msg is None:
                    continue
                if msg.error():
                    if msg.error().code() == KafkaError._PARTITION_EOF:
                        continue
                    else:
                        raise ValueError(f"Kafka Consumer Error: {msg.error()}")
                    
                # callback function
                if callback:
                    callback(msg)
        except KeyboardInterrupt:
           print("Message consuming interrupted by the user.")
        finally:
            self.consumer.close()

    def create_topic(self, topic_name, topic_config, num_partitions=1, replication_factor=1, overwrite=False):
        topics_list = self.list_topics()
        if topic_name in topics_list:
            if overwrite:
                logging.info(f"Topic '{topic_name}' exists. Deleting...")
                self.delete_topic(topic_name)
            else:
                logging.info(f"Topic '{topic_name}' already exists. Skipping creation.")
                return

        new_topic = NewTopic(
            topic=topic_name,
            num_partitions=num_partitions,
            replication_factor=replication_factor,
            config=topic_config,
        )
        
        fs = self.admin_client.create_topics([new_topic])
        for topic, f in fs.items():
            try:
                f.result()
                logging.info(f"Topic '{topic}' created.")
            except Exception as e:
                logging.error(f"Failed to create topic '{topic}': {e}")

    def delete_topic(self,
                     topic_name):
        # Delete the topic
        fs = self.admin_client.delete_topics([topic_name])

        # Block until the deletion is confirmed
        for topic, f in fs.items():
            try:
                f.result()  # The result itself is None
                time.sleep(5)
                print(f"Topic {topic} deleted")
            except Exception as e:
                print(f"Failed to delete topic {topic}: {e}")
        
    def list_topics(self):
        return self.admin_client.list_topics(timeout=10).topics