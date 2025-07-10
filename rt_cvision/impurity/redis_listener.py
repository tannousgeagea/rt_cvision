import os
import redis
import json
import logging

logging.basicConfig(level=logging.INFO)

r = redis.Redis(
    host=os.environ['REDIS_HOST'],
    port=os.environ['REDIS_PORT'],
    db=os.environ['REDIS_DB'],
    password=os.environ['REDIS_PASSWORD'],
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