import os
import json
import logging
import requests
from datetime import datetime, timezone

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

def save_results_into_db(params ):
    data:dict = {}
    success = False
    try:
        
        assert "event_uid" in params, f"Missing argument in save_results_into_db: event_uid"
        assert "delivery_id" in params, f"Missing argument in save_results_into_db: delivery_id"
        assert "objects" in params, f"Missing argument in save_results_into_db: objects"
        assert "snapshot_url" in params, f"Missing argument in save_result_into_db: snapshot_url"
        assert 'snapshot_id' in params, f"Missing argument in save_result_into_db: snapshot_id"
        assert 'db_url' in params, f"Missing argument in save_result_into_db: db_url"
        
        objects = params.get('objects')
        assert "object_uid" in objects, f"key: object_uid not found in objects"
        assert "confidence_score" in objects, f"key: confidence_score not found in objects"
        assert "severity_level" in objects, f"key: severity_level not found in objects"

        
        data.update(
            {
                "request": {
                    'timestamp': datetime.now(tz=timezone.utc).strftime('%Y-%m-%d %H:%M:%S'),
                    'event_uid': params.get('event_uid'),
                    'object_uid': objects.get('object_uid'),
                    'confidence_score': objects.get('confidence_score'),
                    'delivery_id': params.get('delivery_id'),
                    "severity_level": objects.get('severity_level'),
                    "img_file": params.get('snapshot_url'),
                    "img_id": params.get('snapshot_id'), 
                    'model_name': params.get('model_name'),
                    'model_tag': params.get('model_tag'),
                    'meta_info': params.get('meta_info'),       
                }
            }
        )
        
        success = send_request(url=params.get('db_url'), data=data, headers={'Content-Type': 'application/json'},)
    except Exception as err:
        logging.error(f'Error sending request to db: {err}')
    
    return success