from django.db.models.signals import post_save
from django.dispatch import receiver
from .models import Impurity
from impurity.tasks import cvisionops
import logging

@receiver(post_save, sender=Impurity)
def handle_tasks(sender, instance, created, **kwargs):
    """
    Signal to trigger a Celery task whenever a new impurity is detected.
    """
    logging.info(f"Signal Triggered - Created: {created}")
    if created:  # Trigger only for newly created entries
        # process_impurity_event.delay(instance.id) 
        logging.info('Executing Tasks ... ...')
        cvisionops.core.execute.apply_async(args=(instance.id, ), task_id=instance.object_uid)
