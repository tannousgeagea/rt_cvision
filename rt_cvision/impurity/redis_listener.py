import os
import json
import logging
from configure.client import ServiceConfigClient
from common_utils.services.redis import redis_manager
from impurity.tasks.core import TaskExecutor

logging.basicConfig(level=logging.INFO)

if not redis_manager:
    raise ValueError(f"⚠️ Redis is not available.")

r = redis_manager.redis_client 
task_executor = TaskExecutor()

config_client = ServiceConfigClient(
    api_url="http://localhost:23085",
    service_id="impurity"
)

def start_listener():
    logging.info("Redis List listener started.")
    logging.info(f"Available tasks: {list(task_executor.task_registry.keys())}")

    while True:
        try:
            _, raw = r.blpop("impurity:queue")
            data = json.loads(raw)
            impurity_id = data["id"]
            task_id = data.get("object_uid")

        #     # Fan-out to multiple Celery tasks
        #     cvisionops.core.execute.apply_async(args=(impurity_id,), task_id=task_id)
        #     context.core.execute.apply_async(args=(impurity_id,), task_id=task_id)
            
        #     logging.info(f"Triggered tasks for impurity {impurity_id}")

        # except Exception as e:
        #     logging.error(f"Error handling event: {e}")

            tasks_to_execute = data.get("tasks")

            # Execute configured tasks
            result = task_executor.execute_tasks(
                impurity_id=impurity_id,
                task_id=task_id,
                tasks_to_execute=tasks_to_execute
            )
            
            if result['executed']:
                logging.info(f"Successfully executed tasks {result['executed']} for impurity {impurity_id}")
            
            if result['failed']:
                logging.error(f"Failed to execute tasks {result['failed']} for impurity {impurity_id}")
                
        except json.JSONDecodeError as e:
            logging.error(f"Invalid JSON in message: {e}")
        except KeyError as e:
            logging.error(f"Missing required field in message: {e}")
        except Exception as e:
            logging.error(f"Error handling event: {e}")

if __name__ == "__main__":
    start_listener()