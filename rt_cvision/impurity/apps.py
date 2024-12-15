from django.apps import AppConfig
from impurity.config import celery_utils
from impurity.tasks.cvisionops.core import execute


class ImpurityAppConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'impurity'

    def ready(self):
        # Import the signal handlers
        import impurity.signals
        



celery = celery_utils.create_celery()