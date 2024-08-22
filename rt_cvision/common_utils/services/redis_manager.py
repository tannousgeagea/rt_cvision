import cv2
import uuid
import redis
import logging
import numpy as np
from dataclasses import dataclass

@dataclass
class RedisManager:
    """
    A manager class for interfacing with a Redis server for storage and retrieval of images.
    
    Attributes:
        params (dict): A dictionary of connection parameters where keys are parameter names
                       and values are the settings for the Redis server. If not provided, default 
                       settings are used.
        host (str): The host name of the Redis server. Defaults to 'localhost'.
        port (int): The port number of the Redis server. Defaults to 6379.
        db (int): The database number to use. Defaults to 0.
        password (str): The password for the Redis server. Defaults to None.
        expire (int): The expiry time for stored images in seconds. Defaults to 5 seconds.
        redis_client (redis.StrictRedis): The Redis client object used for interacting with the Redis server.
    
    Methods:
        handle_storage(image): Encodes and stores an image in the Redis server.
        retrieve_image(key): Retrieves and decodes an image from the Redis server using a specified key.
    """
    def __init__(
            self,
            params=None,
            host='localhost',
            port=6379,
            db=0,
            password=None,
            ):
        """
        Initializes the RedisManager with connection settings for the Redis server.

        Args:
            params (dict, optional): A dictionary of connection parameters to override default settings.
            host (str, optional): The host name of the Redis server.
            port (int, optional): The port number of the Redis server.
            db (int, optional): The database number to use.
            password (str, optional): The password for the Redis server.
            expire (int, optional): The expiry time for stored images in seconds.
        """
        
        self.params = params if not params is None else {}
        self.host = self.params['host'] if 'host' in self.params.keys() else host
        self.port = self.params['port'] if 'port' in self.params.keys() else port
        self.db = self.params['db'] if 'db' in self.params.keys() else db
        self.password = self.params['password'] if 'password' in self.params.keys() else password

        self.redis_client = redis.StrictRedis(
            host=self.host,
            port=self.port,
            password=self.password,
            db=self.db,
        )

    def handle_storage(self, image, key, expire=5):
        """
        Encodes an image to a JPEG format, stores it in Redis with a unique key, and sets an expiry time.

        Args:
            image (numpy.ndarray): The image to be stored, represented as a NumPy array.

        Returns:
            tuple: A tuple containing:
                - status (bool): True if the storage operation was successful, False otherwise.
                - img_key (str): The unique key under which the image was stored.
        """
        status = False
        img_key = key

        if image is None:
            logging.warning('Warning: Invalid Image')
            return status, img_key
        
        try:   
            _, buffer = cv2.imencode('.jpg', image)
            image_data = np.array(buffer).tobytes()
            
            assert isinstance(expire, int), f'expire expected to be integer but got {type(expire)}'
            assert not image_data is None, f'Image data is None'
            assert isinstance(img_key, str), f'image key expected to be string but gut {type(img_key)}'
 
            self.redis_client.setex(img_key, expire, image_data)
            status = True
        except Exception  as err:
            print("Error while handing storage with redis: %s" %err)
            logging.error("Error while handing storage with redis: %s" %err)

        return status, img_key
    
    
    def handle_storage_by_timestamp(self, image, key, expire, set_name):
        """
        Encodes an image to a JPEG format, stores it in Redis with a unique key, and sets an expiry time.

        Args:
            image (numpy.ndarray): The image to be stored, represented as a NumPy array.

        Returns:
            tuple: A tuple containing:
                - status (bool): True if the storage operation was successful, False otherwise.
                - img_key (str): The unique key under which the image was stored.
        """
        status = False
        img_key = key

        if image is None:
            logging.warning('Warning: Invalid Image')
            return status, img_key
        
        try:   
            _, buffer = cv2.imencode('.jpg', image)
            image_data = np.array(buffer).tobytes()
            
            assert isinstance(expire, int), f'expire expected to be integer but got {type(expire)}'
            assert not image_data is None, f'Image data is None'
            assert isinstance(img_key, str), f'image key expected to be string but gut {type(img_key)}'
 
            self.redis_client.setex(img_key, expire, image_data)
            self.redis_client.zadd(set_name, {img_key: int(img_key)})
            
            status = True
        except Exception  as err:
            print("Error while handing storage with redis: %s" %err)
            logging.error("Error while handing storage with redis: %s" %err)

        return status, img_key
    
    
    
    def retrieve_image(self, key):
        """
        Retrieves an image from Redis using a specified key and decodes it from JPEG format.

        Args:
            key (str): The unique key corresponding to the stored image.

        Returns:
            tuple: A tuple containing:
                - status (bool): True if the retrieval and decode operations were successful, False otherwise.
                - image (numpy.ndarray or None): The retrieved image as a NumPy array, or None if not found or on error.
        """
        status = False
        try:
            image_data = self.redis_client.get(key)
            if image_data is None:
                return status, None
            
            image_array = np.frombuffer(image_data, dtype=np.uint8)
            image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            status = True
        except Exception as err:
            logging.error("Error while retrieving image with redis: %s" %err)

        return status, image
    
    @property
    def memory_info(self,):
        return self.redis_client.info('memory')