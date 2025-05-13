# ml_model_loader.py

from common_utils.model.base import BaseModels

_model_cache = {}

def get_model(weights, mlflow, task):
    key = (weights, mlflow, task)
    if key not in _model_cache:
        print(f"Loading model for key: {key}")
        _model_cache[key] = BaseModels(weights=weights, mlflow=mlflow, task=task)
    return _model_cache[key]
