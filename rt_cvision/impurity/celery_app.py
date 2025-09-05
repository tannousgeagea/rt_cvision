from impurity.config import celery_utils
from impurity.tasks.cvisionops.core import execute as cvision_execute
from impurity.tasks.context.core import execute as context_execute

celery = celery_utils.create_celery()