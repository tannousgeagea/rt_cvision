import logging
import requests
from datetime import datetime, timezone
from requests.exceptions import HTTPError
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

def get(url, params):
    delivery_id = None
    try:
        response = requests.get(
            url=url,
            params=params
        )
        
        if not response.ok:
            logging.error(f"Failed to get delivery id: {response.status_code}: {response.json()}")
            return delivery_id
            
        delivery_id = response.json().get('delivery_uid')
    
    except HTTPError as err:
        logging.error(f"HTTPError getting delivery id: {err}")
    except Exception as err:
        logging.error(f'Error getting delivery id: {err}')
        
    return delivery_id