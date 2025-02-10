from django.db.models.signals import post_save
from django.dispatch import receiver
from .models import Impurity, ImpurityTask
# from impurity.tasks.cvisionops.core import execute as sync_dev
from impurity.tasks import register_alarm
import logging
import uuid

@receiver(post_save, sender=Impurity)
def handle_tasks(sender, instance, created, **kwargs):
    """
    Signal to trigger a Celery task whenever a new impurity is detected.
    """
    logging.info(f"Signal Triggered - Created: {created}")
    if created: 
        if ImpurityTask.objects.filter(name="register_alarm", is_enabled=True).exists():
            register_alarm.core.register_alarm.apply_async(args=(instance.id,), task_id=str(uuid.uuid4()))
        if ImpurityTask.objects.filter(name="notify_users", is_enabled=True).exists():
            notify_users.apply_async(args=(instance.id, ))
        if ImpurityTask.objects.filter(name="sync_dev", is_enabled=True).exists():
            sync_dev.apply_async(args=(instance.id, ), task_id=instance.object_uid)
