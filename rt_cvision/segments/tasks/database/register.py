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

def save_results_into_db(params):
    data:dict = {}
    success = False
    try:
        
        assert 'db_url' in params, f'Missing argument in save_results_into_db: db_url'
        assert 'objects' in params, f'Missing argument in save_results_into_db: objects'
        
        objects = params.get('objects')
        assert 'object_uid' in objects, f'Missing key in objects: object_uid'
        assert 'tracker_id' in objects, f'Missing key in objects: tracker_id'
        # assert 'xyn' in objects, f'Missing key in objects: xyn'
        assert 'object_area' in objects, f'Missing key in objects: object_area'
        assert 'confidence_score' in objects, f'Missing key in objects: confidence_score'
        assert 'object_length' in objects, f'Missing key in objects: object_length'
        
        
        data.update(
            {
                "request": {
                    'timestamp': datetime.now(tz=timezone.utc).strftime('%Y-%m-%d %H:%M:%S'),
                    'object_uid': objects.get('object_uid'),
                    'object_tracker_id': objects.get('tracker_id'),
                    'object_polygon': [None] * len(objects.get('object_uid')), #objects.get('xyn'),
                    'confidence_score': objects.get('confidence_score'),
                    'object_area': objects.get('object_area'),
                    'object_length': objects.get('object_length'),
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


