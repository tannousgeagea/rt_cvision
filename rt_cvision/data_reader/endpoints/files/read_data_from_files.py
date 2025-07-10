import os
import cv2
import time
import logging
from datetime import time as ti
from datetime import datetime, timezone
from data_reader.utils.file_utils import examine_src
from common_utils.services.redis import redis_manager

VALID_FORMAT = ['jpeg', 'png', 'jpg', 'webp']
DEFAULT_ACQUISITION_RATE = 1 #fps
DATETIME_FORMAT = "%Y-%m-%d %H-%M-%S"

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
                if redis_manager:
                    raw = redis_manager.redis_client.get("ACQUISITION_RATE")
                else:
                    raw = DEFAULT_ACQUISITION_RATE
                try:
                    ACQUISITION_RATE = float(raw) if raw else DEFAULT_ACQUISITION_RATE
                except (ValueError, TypeError):
                    ACQUISITION_RATE = DEFAULT_ACQUISITION_RATE
                    
                logging.info(f'Publishing at {ACQUISITION_RATE} fps')
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
                time.sleep(1 / (int(ACQUISITION_RATE) + 1))
    except Exception as err:
        logging.error(f'Error reading data from files: {err}')

