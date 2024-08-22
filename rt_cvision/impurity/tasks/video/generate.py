import json
import logging
import requests
from typing import Dict


def generate_video(params:Dict):
    success = False
    try:
        assert 'timestamp' in params, f'Missing argument in generate_video: timestamp'
        assert 'severity_level' in params, f'Missing argument in generate_video: severity_level'
        assert 'event_uid' in params, f'Missing argument in generate_video: event_uid'
        assert 'event_description' in params, f'Missing argument in generate_video: event_description'
        assert 'roi' in params, f'Missing argument in generate_video: roi'
        assert 'roi_color' in params, f'Missing argument in generate_video: roi_color'
        assert 'video_url' in params, f'Missing argument in generate_video: video_url'

        result = {
            'event_type': 'Störstoff',
            'timestamp': str(params.get('timestamp')),
            'event_id': params.get('event_uid'),
            'event_name': 'StoerstofDetection',
            'event_severity_level': params.get('severity_level'),
            'event_description': params.get('event_description'),
            'event_roi': params.get('roi'),
            'roi_color': params.get('roi_color')
        }
        
        response = requests.post(params.get('video_url'), data=json.dumps(result), headers={'Content-Type': 'application/json'})
        response.raise_for_status()
        success = True
    except requests.RequestException as e:
        logging.error(f"Error posting results to video API: {e}")
        
    return success
