import json
import requests
import logging

def execute(
        params:dict,
):
    success = False
    try:
        assert 'severity_level' in params, f'Missing argument in post_email: severity_level'
        assert 'event_uid' in params, f'Missing argument in post_email: event_uid'
        assert 'pc_url' in params, f'Missing argument in post_email: pc_url'

        sv = max(params['severity_level'])
        if int(sv) < 3:
            return success

        result = {
            "tenant_domain": params.get('tenant_domain'),
            "event_type": "Störstoff",
            "severity_level": str(sv),
            "location": params.get('tenant_location'),
            'region': params.get('location_loc'),
        }

        response = requests.post(
            params["pc_url"], 
            data=json.dumps(result), 
            headers={
                "accept": "application/json",
                'Content-Type': 'application/json'
                })
        response.raise_for_status()

    except requests.RequestException as e:
        logging.error(f"Error sending request to Plant Controller: {e}")