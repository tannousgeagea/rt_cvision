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
   
        sv = max(params['severity_level'])
        if not int(sv) > 1:
            return success
        
        timestamp = params.get("timestamp")
        if isinstance(timestamp, datetime):
            timestamp = timestamp.strftime(DATETIME_FORMAT)
            
        objects = params.get("objects", {})
        outliers = [round(o * 100) for o in objects.get("object_length", [])]

        result = {
            "tenant_domain": params['tenant_domain'],
            "event_type": "Störstoff", 
            "timestamp": timestamp, 
            "severity_level": severity_level_map[str(sv)], 
            "location": params.get('tenant_location'), 
            'image_url': params.get('snapshot_url'),
            'delivery_id': params.get('delivery_id'),
            'region': params.get('location_loc'),
            'alarm_description': f"{len(outliers)} prob. Langteile: {outliers} cm",
            }
        
        headers = {
            "accept": "application/json",
            "Content-Type": "application/json"
        }
        

        logging.info(result)
        response = requests.post(params.get('email_url'), data=json.dumps(result), headers=headers)
        print("Status Code:", response.status_code)
        print("Response Body:", response.text)
        response.raise_for_status()

    except requests.RequestException as e:
        logging.error(f"Error posting results to email API: {e}")
        
    return success



