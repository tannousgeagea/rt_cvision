import os
import sys
from pathlib import Path
import logging
from django.apps import AppConfig
from impurity.config import celery_utils
from impurity.tasks.cvisionops.core import execute
from impurity.tasks.register_alarm.core import register_alarm

class ImpurityAppConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'impurity'

    def ready(self):
        # Import the signal handlers
        import impurity.signals
        



celery = celery_utils.create_celery()