import json
import logging
import requests
from datetime import datetime

DATETIME_FORMAT = "%Y-%m-%d %H:%M:%S"

severity_level_map = {
    '1': 'Niedrig',
    '2': 'Mittel',
    '3': 'Hoch'
}


def send_email(params):
    success = False
    try:
        assert 'timestamp' in params, f'Missing argument in post_email: timestamp'
        assert 'severity_level' in params, f'Missing argument in post_email: severity_level'
        assert 'event_uid' in params, f'Missing argument in post_email: event_uid'
        assert 'event_description' in params, f'Missing argument in post_email: event_description'
        assert 'snapshot_url' in params, f'Missing argument in post_email: url_img_name'
        assert 'email_url' in params, f'Missing argument in post_email: email_url'
        
        if not params['severity_level'] > 1:
            return success
        
        if isinstance(timestamp, datetime):
            timestamp = timestamp.strftime(DATETIME_FORMAT)
            
        result = {
            "event_type": "Störstoff", 
            "event_timestamp": timestamp, 
            "event_severity_level": severity_level_map[str(params.get("severity_level"))], 
            "event_location": params.get('location'), 
            "plant_id": params.get('plant_id'),
            'snapshot': params.get('snapshot_url'),
            'delivery_id': params.get('delivery_id'),
            'region': params.get('delivery_region'),
            'event_description': params.get('event_description'),
            }
        
        response = requests.post(params.get('email_url'), data=json.dumps(result), headers={'Content-Type': 'application/json'})
        response.raise_for_status()

    except requests.RequestException as e:
        logging.error(f"Error posting results to email API: {e}")
        
    return success



