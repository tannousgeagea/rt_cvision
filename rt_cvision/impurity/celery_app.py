from impurity.config import celery_utils
from impurity.tasks.cvisionops.core import execute

celery = celery_utils.create_celery()