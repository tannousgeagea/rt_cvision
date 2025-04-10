import requests
import json
import logging
from common_utils.api.base import BaseAPI
base_api = BaseAPI()

def sync(
    url:str,
    params:dict,
    media_file,
):
    try:
        with open(media_file, 'rb') as file:
            files = {
                'media_file': file
            }

            print(params)
            response = requests.post(url, params=params, files=files)

        # Check if the request was successful
        if response.status_code == 200:
            logging.info("File successfully uploaded:", response.json())
        else:
            raise ValueError(f"Failed to upload file. Status code: {response.status_code}, Response: {response.text}")

    except requests.exceptions.RequestException as e:
        raise ValueError(f"An error occurred: {e}")
    except Exception as e:
        raise ValueError(f"An unexpected error occurred: {e}")

def sync_to_alarm(
    url:str, 
    params:dict,  
    ):
    try:
        sv = max(params['severity_level'])
        delivery_id = params.get("delivery_id")
        base_api.post(
            url=url,
            payload={
                'event_id': params.get("event_uid"),
                "source_id": "impurity",
                "target": "alarm",
                "data": {
                    "tenant_domain": params.get("tenant_domain"),
                    "delivery_id": delivery_id if delivery_id is not None else "",
                    "location": params.get("location"),
                    "flag_type": f"impurity",
                    "severity_level": str(sv),
                    "timestamp": params.get("timestamp"),
                    "event_uid": params.get("event_uid"),
                    "meta_info": params.get("meta_info"),
                }
            }
        )
        
        if params.get("delivery_id"):
            base_api.post(
                url=url,
                payload={
                    'event_id': params.get("tenant_domain"),
                    "source_id": "impurity",
                    "target": "delivery/flag",
                    "data": {
                        "delivery_id": params.get("delivery_id"),
                        "flag_type": "impurity",
                        "severity_level": str(sv),
                        "event_uid": params.get("event_uid"),
                    }
                }
            )
    except Exception as err:
        raise ValueError(f"Error in sync: {err}")