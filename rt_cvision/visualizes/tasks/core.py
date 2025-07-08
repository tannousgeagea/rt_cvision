import time
import asyncio
import logging
from typing import Callable, Dict, Any, Awaitable, List

class TaskRunner:
    def __init__(self, tasks: Dict[str, Callable[[Dict[str, Any]], None]]):
        """
        Args:
            tasks (dict): Mapping of task name to callable functions.
        """
        self.tasks = tasks
        logging.info("Tasks: ")
        logging.info("------------------")
        for task in self.tasks:
            logging.info(f"\t {task}")
        logging.info("\n")

    def run(self, tasks:List[str], parameters: Dict[str, Any]) -> Dict[str, float]:
        """
        Executes tasks conditionally with timing and exception handling.

        Args:
            parameters (dict): Includes 'tasks' key with task flags, plus shared `params`.

        Returns:
            Dict[str, float]: A dictionary mapping task names to execution time in milliseconds.
        """
        results = {}
        shared_params = parameters  # Pass entire parameter set to tasks

        for task in tasks:
            func = self.tasks.get(task)
            if not func:
                continue

            start_time = time.time()
            logging.info(f"▶ Executing task: {task}...")

            try:
                func(shared_params)
                logging.info(f"✅ Completed task: {task}")
            except Exception as e:
                logging.exception(f"❌ Error during task: {task} - {e}")
            finally:
                elapsed = round((time.time() - start_time) * 1000, 2)
                results[task] = elapsed
                logging.info(f"⏱ Task {task} took {elapsed} ms")

        return results


def wrap_sync(sync_func):
    async def wrapper(params):
        await asyncio.to_thread(sync_func, params)
    return wrapper