from abc import ABC, abstractmethod

import numpy as np
from common_utils.detection.core import Detections


class BaseTracker(ABC):
    @abstractmethod
    def update(self, detections: Detections) -> Detections:
        pass

    @abstractmethod
    def reset(self) -> None:
        pass


class BaseTrackerWithFeatures(ABC):
    @abstractmethod
    def update(self, detections: Detections, frame: np.ndarray) -> Detections:
        pass

    @abstractmethod
    def reset(self) -> None:
        pass
