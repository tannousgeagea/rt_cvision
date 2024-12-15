import os
import time
import requests
from celery import shared_task
import logging
from datetime import datetime
logger = logging.getLogger(__name__)
from common_utils.sync.core import sync

@shared_task(
    bind=True,
    autoretry_for=(Exception,),
    retry_backoff=True,
    retry_kwargs={"max_retries": 5},
    ignore_result=True,
    name='impurity:execute'
)
def execute(self, instance, **kwargs):
    """
    Task to execute an alarm action based on the given alarm_id.

    :param self: Celery task instance
    :param alarm_id: The ID of the alarm to process
    """
    start_time = time.time()
    import django
    django.setup()
    from impurity.models import Impurity
    data:dict = {}
    try:
        logger.info(f"Executing alarm with ID: {instance}")
        wi = Impurity.objects.get(id=instance)
        image = wi.image
        media_file = image.image_file.path
        print(media_file)
        if not os.path.exists(media_file):
            raise ValueError(f"{media_file} does not exist")
        
        sync(
            url = "http://10.10.0.7:29085/api/v1/images",
            params = {
                "source_of_origin": "testing"
            },
            media_file=media_file,
        )
        
    except requests.exceptions.RequestException as e:
        raise ValueError(f"An error occurred: {e}")
    except Exception as e:
        raise ValueError(f"An unexpected error occurred: {e}")

        data.update(
            {
                'action': 'done',
                'time':  datetime.now().strftime("%Y-%m-%d %H-%M-%S"),
                'result': 'success',
                'execution time': f"{round((time.time() - start_time) * 1000, 2)} ms",
            }
        )
        
    except Exception as e:
        # Log the error and let Celery handle the retry
        logger.error(f"Error executing alarm {instance}: {e}")
        raise e

    return data