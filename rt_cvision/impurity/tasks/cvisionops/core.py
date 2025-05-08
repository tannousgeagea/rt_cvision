import os
import cv2
import time
import requests
from celery import shared_task
import logging
from datetime import datetime
logger = logging.getLogger(__name__)
from common_utils.sync.core import sync
from common_utils.model.base import BaseModels
# from configure.client import config_manager
# parameters = config_manager.params.get('impurity')

# model = BaseModels(
#     weights=parameters.get('weights'), task='impurity', mlflow=parameters.get('mlflow', {}).get('active', False)
# )

CVISIONOPS_API_URL = "http//cvisionops.want:29085"
CVISIONOPS_PROJECT_NAME = "amk_front_impurity"


def post_annotations(api_url, project_name, image_id, annotation_type, annotations):
    """
    Posts annotations to the specified API endpoint.

    Args:
        api_url (str): Base URL of the API.
        project_name (str): Name of the project.
        image_id (str): ID of the image.
        annotation_type (str): Type of annotation (e.g., 'bounding_boxes').
        annotations (list): List of annotations to be sent in the request body.

    Returns:
        Response: Response object from the API.
    """
    url = f"{api_url}/api/v1/annotations"
    params = {
        "project_name": project_name,
        "image_id": image_id,
        "annotation_type": annotation_type,
    }
    headers = {
        "accept": "application/json",
        "Content-Type": "application/json",
    }
    
    print(annotations)
    try:
        response = requests.post(url, headers=headers, params=params, json=annotations)
        response.raise_for_status()  # Raise an exception for HTTP errors
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error posting annotations: {e}")
        return None


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
        if not os.path.exists(media_file):
            raise ValueError(f"{media_file} does not exist")
        
        url = f"{CVISIONOPS_API_URL}/api/v1/images"
        params = {
            "source_of_origin": wi.image.sensorbox.sensor_box_name,
            "image_id": wi.image.image_id,
        }
        
        with open(media_file, "rb") as file:
            files = {
                "files": file
            }
            response = requests.post(url, params=params, files=files)

        if response.status_code == 200:
            print("File successfully uploaded:", response.json())
        else:
            raise ValueError(f"Failed to upload file. Status code: {response.status_code}, Response: {response.text}")


        cv_image = cv2.imread(image.image_file.path)
        detections = model.classify_one(
            image=cv_image,
            conf=parameters.get('conf', 0.25),
            mode="detect"
        )

        for i, xyxyn in detections.xyxyn:
            post_annotations(
                api_url=f"{CVISIONOPS_API_URL}",
                project_name = f"{CVISIONOPS_PROJECT_NAME}",
                image_id=wi.image.image_id,
                annotation_type='bounding_boxes',
                annotations=[[int(detections.class_id[i])] + xyxyn.tolist()],
                # annotations=[[wi.class_id] + wi.object_coordinates],
            )

        data.update(
            {
                'action': 'done',
                'time':  datetime.now().strftime("%Y-%m-%d %H-%M-%S"),
                'result': 'success',
                'execution time': f"{round((time.time() - start_time) * 1000, 2)} ms",
            }
        )

    except requests.exceptions.RequestException as e:
        raise ValueError(f"An error occurred: {e}")
    except Exception as e:
        raise ValueError(f"An unexpected error occurred: {e}")

    return data