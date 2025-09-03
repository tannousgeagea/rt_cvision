
import uuid
import json
import logging
import requests
from datetime import datetime, timezone
from typing import Dict


def generate_video(params:Dict):
    success = False
    try:
        assert 'event_uid' in params, f'Missing argument in generate_video: event_uid'
        assert 'event_description' in params, f'Missing argument in generate_video: event_description'
        assert 'gate_id' in params, f"Missing argument in generate_video: gate_id"
        assert 'video_url' in params, f'Missing argument in generate_video: video_url'
        assert 'topic' in params, f'Missing argument in generate_video: topic'

        result = {
            'gate_id': params.get('gate_id'),
            'event_type': 'Störstoff',
            'timestamp': datetime.now(tz=timezone.utc),
            'event_id': params.get('event_uid'),
            'event_name': 'StoerstofDetection',
            'event_description': params.get('event_description'),
            'topic': params.get('topic'),
        }
        
        response = requests.post(params.get('video_url'), params=result, headers={'Content-Type': 'application/json'})
        response.raise_for_status()
        success = True
    except requests.RequestException as e:
        logging.error(f"Error posting results to video API: {e}")
        
    return success
