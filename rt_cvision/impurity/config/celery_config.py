import os
import celery
from functools import lru_cache
from kombu import Queue

REDIS_HOST = os.environ.get("REDIS_HOST", "localhost")
REDIS_PORT = os.environ.get("REDIS_PORT", "6379")
REDIS_DB = os.environ.get("REDIS_DB", "0")  # Default Redis database
REDIS_PASSWORD = os.environ.get("REDIS_PASSWORD", "") 

def route_task(name, args, kwargs, options, task=None, **kw):
    print(name)
    if ":" in name:
        queue, _ = name.split(":")
        return {"queue": queue}
    return {"queue": "celery"}

class BaseConfig:
    
    CELERY_BROKER_URL: str = os.environ.get(
        "CELERY_BROKER_URL", f"redis://:{REDIS_PASSWORD}@{REDIS_HOST}:{REDIS_PORT}/{REDIS_DB}"
    )
    CELERY_RESULT_BACKEND: str = os.environ.get(
        "CELERY_RESULT_BACKEND", f"redis://:{REDIS_PASSWORD}@{REDIS_HOST}:{REDIS_PORT}/{REDIS_DB}"
    )


    CELERY_TASK_QUEUES: list = (
        # default queue
        Queue("celery"),
        # custom queue
    )

    CELERY_TASK_ROUTES = (route_task,)
    ACCEPT_CONTENT = ['json', 'pickle']
    TASK_SERIALIZE = 'pickle'
    RESULT_SERIALIZE = 'pickle'
    TIMEZONE = 'UTC'
    ENABLE_UTC = True 

class DevelopmentConfig(BaseConfig):
    pass


@lru_cache()
def get_settings():
    config_cls_dict = {
        "development": DevelopmentConfig,
    }
    config_name = os.environ.get("CELERY_CONFIG", "development")
    config_cls = config_cls_dict[config_name]
    return config_cls()


settings = get_settings()
