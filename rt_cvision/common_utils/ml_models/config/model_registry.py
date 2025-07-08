from common_utils.ml_models.inference.yolo.segmentation.core import YOLOSegInferencePlugin
from common_utils.ml_models.inference.yolo.detection.core import YOLODetInferencePlugin

MODEL_REGISTRY = {
    ("detection", "yolo"): YOLODetInferencePlugin,
    ("segmentation", "yolo"): YOLOSegInferencePlugin,
}
