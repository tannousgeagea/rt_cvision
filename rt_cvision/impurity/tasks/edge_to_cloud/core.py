import os
import json
import uuid
import logging
from common_utils.sync.core import sync, sync_to_alarm


def sync_media_to_cloud(
    params:dict
):
    
    try:
        assert 'edge_2_cloud_url' in params, f'Missing argument in sync_media_to_cloud: edge_2_cloud_url'
        assert 'event_uid' in params, f'Missing argument in sync_media_to_cloud: event_uid'
        assert 'media_file' in params, f"Missing argument in sync_media_to_cloud: media_file"
        assert 'filename' in params, f'Missing argument in sync_media_to_cloud: filename'

        if params.get("edge_2_cloud_url_data"):
            sync_to_alarm(
                url=params['edge_2_cloud_url_data'],
                params=params,
            )

        sync(
            url=params['edge_2_cloud_url'],
            media_file=params['media_file'],
            params={
                "event_id": params['event_uid'],
                "source_id": "impurity",
                "blob_name": params["filename"],
                "container_name": "alarm/impurity",
                "target": "alarm/media",
                "data": json.dumps(
                    {
                        "event_uid": params['event_uid'],
                        "media_id": params.get('snapshot_id', str(uuid.uuid4())),
                        "media_name": params['filename'],
                        "media_type": 'image',
                        "media_url": params.get('snapshot_url', '/alarm/impurity'),               
                    }
                )
            }
        )
        
    except Exception as err:
        logging.error(
            f"Error syncing data to cloud: {err}"
        )