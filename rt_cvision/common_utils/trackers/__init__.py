from common_utils.trackers.core.sort.tracker import SORTTracker
from common_utils.trackers.log import get_logger

__all__ = ["SORTTracker"]

logger = get_logger(__name__)

try:
    from common_utils.trackers.core.deepsort.tracker import DeepSORTTracker
    from common_utils.trackers.core.reid.model import ReIDModel

    __all__.extend(["DeepSORTTracker", "ReIDModel"])
except ImportError:
    logger.warning(
        "ReIDModel dependencies not installed. ReIDModel will not be available. "
        "Please run `pip install trackers[reid]` and try again."
    )
    pass
