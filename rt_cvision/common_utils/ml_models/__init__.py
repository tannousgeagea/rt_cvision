from .config.model_registry import MODEL_REGISTRY
from .inference.base import BaseInferencePlugin

def load_inference_module(config: dict):
    key = (config["type"], config["framework"])
    model_class = MODEL_REGISTRY.get(key)
    if not model_class:
        raise ValueError(f"Unsupported type/framework: {key}")
    return model_class
