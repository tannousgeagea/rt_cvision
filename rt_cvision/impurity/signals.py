# from django.db.models.signals import post_save
# from django.dispatch import receiver
# from .models import Impurity
# from impurity.tasks import cvisionops
# import logging

# @receiver(post_save, sender=Impurity)
# def handle_tasks(sender, instance, created, **kwargs):
#     """
#     Signal to trigger a Celery task whenever a new impurity is detected.
#     """
#     logging.info(f"Signal Triggered - Created: {created}")
#     if created:  # Trigger only for newly created entries
#         # process_impurity_event.delay(instance.id) 
#         logging.info('Executing Tasks ... ...')
#         cvisionops.core.execute.apply_async(args=(instance.id, ), task_id=instance.object_uid)


# impurity/utils.py or signals.py
import json
import os
import redis

r = redis.Redis(
    host=os.environ['REDIS_HOST'],
    port=os.environ['REDIS_PORT'],
    db=os.environ['REDIS_DB'],
    password=os.environ['REDIS_PASSWORD'],
)

def queue_impurity_event(instance):
    data = {
        "event": "impurity",
        "id": instance.id,
        "object_uid": instance.object_uid,
        "created_at": instance.created_at.strftime('%Y-%m-%d %H:%M:%S'),
    }
    
    print(data)
    r.rpush("impurity:queue", json.dumps(data))
