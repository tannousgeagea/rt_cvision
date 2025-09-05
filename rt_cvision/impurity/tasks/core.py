
import os
import json
import time
import asyncio
import logging
import importlib
from configure.client import ServiceConfigClient
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

class TaskExecutor:
    """Dynamic task executor that can load and execute tasks based on configuration."""
    
    def __init__(self):
        self.task_registry = {}
        self._register_available_tasks()
        self.config_client = ServiceConfigClient(
            api_url="http://localhost:23085",
            service_id="impurity"
        )
    
    def _register_available_tasks(self):
        """Dynamically discover and register available tasks."""
        task_modules = [
            'cvisionops',
            'context',
            # Add other task modules here as they become available
        ]
        
        for module_name in task_modules:
            try:
                # Import the task module
                task_module = importlib.import_module(f'impurity.tasks.{module_name}')
                
                # Check if the module has the expected structure (core.execute)
                if hasattr(task_module, 'core') and hasattr(task_module.core, 'execute'):
                    self.task_registry[module_name] = task_module.core.execute
                    logging.info(f"Registered task: {module_name}")
                else:
                    logging.warning(f"Task module {module_name} doesn't have expected structure (core.execute)")
                    
            except ImportError as e:
                logging.warning(f"Could not import task module {module_name}: {e}")
    
    def get_task_config(self) -> List[str]:
        """Get the list of tasks to execute from configuration."""
        try:
            # Try to get from service config first
            config = self.config_client.load()
            if config and 'tasks' in config:
                return config['tasks']
        except Exception as e:
            logging.warning(f"Could not get config from service: {e}")
        
        # Fallback to environment variable
        tasks_env = os.getenv('IMPURITY_TASKS', '[]')
        try:
            return json.loads(tasks_env)
        except json.JSONDecodeError:
            logging.warning("Invalid JSON in IMPURITY_TASKS environment variable")
        
        # Default fallback - execute all available tasks
        return list(self.task_registry.keys())
    
    def execute_tasks(self, impurity_id: str, task_id: str = None, tasks_to_execute: List[str] = None):
        """Execute specified tasks for the given impurity_id."""
        if tasks_to_execute is None:
            tasks_to_execute = self.get_task_config()
        
        executed_tasks = []
        failed_tasks = []
        
        for task_name in tasks_to_execute:
            if task_name not in self.task_registry:
                logging.error(f"Task '{task_name}' is not registered. Available tasks: {list(self.task_registry.keys())}")
                failed_tasks.append(task_name)
                continue
            
            try:
                task_func = self.task_registry[task_name]
                task_func.apply_async(args=(impurity_id,), task_id=task_id)
                executed_tasks.append(task_name)
                logging.info(f"Triggered task '{task_name}' for impurity {impurity_id}")
            except Exception as e:
                logging.error(f"Error executing task '{task_name}': {e}")
                failed_tasks.append(task_name)
        
        return {
            'executed': executed_tasks,
            'failed': failed_tasks
        }