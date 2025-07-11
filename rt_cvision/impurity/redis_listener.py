import os
import json
import logging
from configure.client import ServiceConfigClient
from common_utils.services.redis import redis_manager

logging.basicConfig(level=logging.INFO)

if not redis_manager:
    raise ValueError(f"⚠️ Redis is not available.")

r = redis_manager.redis_client 
config_client = ServiceConfigClient(
    api_url="http://localhost:23085",
    service_id="impurity"
)

def start_listener():
    logging.info("Redis List listener started.")
    while True:
        _, raw = r.blpop("impurity:queue")
        from impurity.tasks import (
            cvisionops
        )
        try:
            data = json.loads(raw)
            impurity_id = data["id"]
            task_id = data.get("object_uid")

            # Fan-out to multiple Celery tasks
            cvisionops.core.execute.apply_async(args=(impurity_id,), task_id=task_id)

            logging.info(f"Triggered tasks for impurity {impurity_id}")

        except Exception as e:
            logging.error(f"Error handling event: {e}")

if __name__ == "__main__":
    start_listener()