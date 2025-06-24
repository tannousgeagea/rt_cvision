from abc import ABC, abstractmethod
from PIL import Image

class BaseInferencePlugin(ABC):
    def __init__(self, weights, device: str = "cuda", config=None):
        self.weights = weights
        self.device = device
        self.config = config or {}

    @abstractmethod
    def predict(self, image: Image.Image, confidence_threshold:float=0.25) -> dict:
        pass