import os
import cv2
import time
import logging
from datetime import time as ti
from datetime import datetime, timezone
from data_reader.utils.file_utils import examine_src
from common_utils.services.redis_manager import RedisManager

VALID_FORMAT = ['jpeg', 'png', 'jpg', 'webp']
src = '/home/appuser/data/images'
DEFAULT_ACQUISITION_RATE = 3 #fps
DATETIME_FORMAT = "%Y-%m-%d %H-%M-%S"

redis_manager = RedisManager(
    host=os.environ['REDIS_HOST'],
    port=os.environ['REDIS_PORT'],
    db=os.environ['REDIS_DB'],
    password=os.environ['REDIS_PASSWORD'],
)

def read_data(params, callback=None):
    payload = {}
    processed = []
    if callback is None:
        def callback(payload):
            logging.info(
                
                'Please define a custom callback!' + \
                    f'\n current payload contains {tuple(payload.keys())}'
                )
    try:
        
        assert "src" in params.keys(), f"key: src not found in params"
        while True:
            images = examine_src(params["src"], VALID_FORMAT)
            for image in images:
                ACQUISITION_RATE = redis_manager.redis_client.get("ACQUISITION_RATE") or DEFAULT_ACQUISITION_RATE
                logging.info(f'Publishing at {int(ACQUISITION_RATE)} fps')
                logging.info(f'{len(images)} Found : {len(processed)} Processed !')
                if image in processed:
                    logging.info(f'{image} already processed')
                    continue
                
                filename = os.path.basename(image)
                file_ext = filename.split('.')[-1]
                filename = filename.split(f'.{file_ext}')[0]
                if os.path.exists(f'/home/appuser/data/labels/{filename}.txt'):
                    logging.info(f'/home/appuser/data/labels/{filename}.txt already exists')
                    continue
                
                cv_image = cv2.imread(image)
                payload = {
                    'filename': os.path.basename(image),
                    'timestamp': datetime.now().timestamp(),
                    'cv_image': cv_image,
                    'datetime': datetime.now().strftime(DATETIME_FORMAT),
                }
                
                callback(payload)
                processed.append(image)
                time.sleep(1 / int(ACQUISITION_RATE))
    except Exception as err:
        logging.error(f'Error reading data from files: {src}: {err}')

