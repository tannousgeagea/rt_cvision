import os
import sys
import json
import logging
import requests
import django
from celery import shared_task
from datetime import datetime, timezone
from pathlib import Path

def send_request(url:str, data:dict, headers:dict):
    success = False
    try:
        response = requests.post(
            url=url,
            data=json.dumps(data),
            headers=headers,   
        )
        
        response.raise_for_status()
        success = True
    except requests.RequestException as e:
        logging.error(f"Error sending result to Waste DB' API: {e}")

    return success

@shared_task(bind=True, autoretry_for=(Exception,), retry_backoff=True, retry_kwargs={"max_retries": 5},ignore_result=True,
    name='impurity:register_alarm'
)
def register_alarm(self, instance_id, **kwargs):
    data:dict = {}
    success = False
    try:
        # django.setup()
        from configure.client import ConfigManager
        from impurity.models import Impurity
        config_manager = ConfigManager()
        wi = Impurity.objects.get(id=instance_id)
        data.update(
            {
                "request": {
                    'timestamp': datetime.now(tz=timezone.utc).strftime('%Y-%m-%d %H:%M:%S'),
                    'event_uid': wi.image.image_id,
                    'object_uid':[ wi.object_uid],
                    'confidence_score': [wi.confidence_score],
                    'delivery_id': None,
                    "severity_level": [wi.class_id],
                    "img_file": wi.image.image_file.url,
                    "img_id": wi.image.image_id, 
                    'model_name': "impurity",
                    'model_tag': "v1",      
                }
            }
        )
        
        success = send_request(url=config_manager.impurity.db_url, data=data, headers={'Content-Type': 'application/json'},)
    except Exception as err:
        raise ValueError(f'Error sending request to db: {err}')
    
    return success