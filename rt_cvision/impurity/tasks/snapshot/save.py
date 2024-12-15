import os
import cv2

import logging
from django.core.files.base import ContentFile

from datetime import (
    datetime,
    timezone
)

from data_reader.models import (
    Image
)

from impurity.models import (
    Impurity,
)


def save_snapshot(params):
    try:
        assert 'snapshot' in params, f"Missing argument in save_snapshot: snapshot"
        assert 'snapshot_dir' in params, f"Missing argument in save_snapshot: snapshot_dir"
        assert 'filename' in params, f"Missing argument in save_snapshot: filename"
        
        os.makedirs(params.get('snapshot_dir'), exist_ok=True)
        cv2.imwrite(f"{params.get('snapshot_dir')}/{params.get('filename')}", params.get('snapshot'))
        
    except Exception as err:
        logging.error(f"Error saving impurity snapshots: {err}")
        
        
def save_experiment(params):
    try:
        assert 'cv_image' in params, f"Missing argument in save_snapshot: cv_image"
        assert 'experiment_dir' in params, f"Missing argument in save_snapshot: experiment_dir"
        assert 'filename' in params, f"Missing argument in save_snapshot: filename"
        
        # os.makedirs(params.get('experiment_dir'), exist_ok=True)
        # cv2.imwrite(f"{params.get('experiment_dir')}/{params.get('filename')}", params.get('cv_image'))
        
        image = Image(
            image_id=params.get('event_uid'),
            image_name=params.get('filename'),
            image_format='JPEG',
            timestamp=datetime.now(tz=timezone.utc),
        )
        
        _, buffer = cv2.imencode('.jpg', params.get('cv_image'))
        image_binary = buffer.tobytes()
        
        image.image_file.save(
            params.get('filename'), 
            ContentFile(image_binary)
            )
        image.save()
        
        objects = params.get('objects')
        for i, xyxyn in enumerate(objects.get('xyxyn')):
            wi = Impurity.objects.create(
                image=image,
                object_uid=objects.get('object_uid')[i],
                timestamp=datetime.now(tz=timezone.utc),
                confidence_score=objects.get('confidence_score')[i],
                class_id=objects.get('severity_level')[i],
                object_length=objects.get('object_length')[i],
                object_coordinates=xyxyn,
            )
            # wi.save()
                    
    except Exception as err:
        logging.error(f"Error saving impurity experiment images {params.get('filename')}: {err}")