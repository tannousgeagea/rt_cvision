import os
import uuid
import logging
import numpy as np
import torch
from common_utils.detection.utils import box_iou_batch
from common_utils.model.base import BaseModels, Detections
from common_utils.object_size.core import ObjectSizeBase
from common_utils.filters.core import FilterEngine

class SeverityLevelXDetector:
    def __init__(self, model_path: str = None, mlflow:bool = False, conf_threshold:float=0.25, X:int=2, cf:float=0.001, filter_config:dict=None):
        self.model = None
        self.conf_threshold = conf_threshold
        self.object_size_est = ObjectSizeBase()
        self.X = X
        self.cf = cf
        if model_path and os.path.exists(model_path):
            try:
                self.model = self.load_model(model_path, mlflow=mlflow)
                logging.info("Severity Level 2 model loaded successfully.")
            except Exception as err:
                logging.error(f"Failed to load Severity Level 2 model: {err}")
        elif mlflow:
            self.model = self.load_model(model_path, mlflow=mlflow)
        else:
            logging.warning("Severity Level 2 model path not provided or file does not exist.")

        self.filter_config = filter_config
        self.filter_engine = FilterEngine()
        if filter_config:
            for obj_type, filter_model in filter_config.items():
                self.filter_engine.add_model(
                    object_type=obj_type,
                    detection_model=filter_model["model_path"],
                    mlflow=filter_model["mlflow"],
                    conf_threshold=filter_model.get('conf_threshold', 0.25),
                )
    
    def load_model(self, path: str, mlflow=True):
        return BaseModels(weights=path, mlflow=mlflow)
    
    def infer(self, full_image: np.ndarray,) -> Detections:
        """
        Run the secondary detection model on the full image.
        Expected output format is a dict with:
            - 'xyxyn': list of bounding boxes in normalized format,
            - 'scores': list of confidence scores,
            - 'class_id': list of predicted class ids (for this model, we expect severe objects, e.g. level 2).
        """
        if self.model is None:
            return Detections.from_dict({})
        try:
            detections = self.model.classify_one(full_image, conf=self.conf_threshold, mode='detect')
            detections.class_id = np.array([self.X] * len(detections))
            detections.uid = np.array([str(uuid.uuid4()) for _ in range(len(detections))])
            detections.tracker_id = np.array([np.random.randint(low=0, high=9999999) for _ in range(len(detections))])
            detections.object_area = detections.box_area
            detections.object_length = np.array(
                self.object_size_est.compute_object_length_bbox(
                    bboxes=detections.xyxyn,
                    input_shape=full_image.shape,
                    correction_factor=self.cf,
                )[1])
            return detections
        except Exception as err:
            logging.error(f"Error during secondary model inference: {err}")
            return Detections.from_dict({})
    
    def integrate(self, primary_detections: dict, full_image: np.ndarray, iou_threshold: float = 0.5) -> dict:
        """
        Integrates the secondary model detections with the primary detections.
        - Runs the secondary detector on the full image.
        - For each secondary detection:
            - If it matches (IoU > threshold) a primary detection, overwrite that detection.
            - If it doesn't match any primary detection, add it as a new detection.
        Returns: A dictionary containing the merged detections.
        """
        secondary_results = self.infer(full_image)
        if not len(secondary_results):
            logging.info("No secondary detections available.")
            return primary_detections


        logging.info(f"Secondary Detection: {secondary_results}")
        if self.filter_config:
            filtered_results = self.filter_engine.filter_objects(
                image=full_image,
                segmentation_results=secondary_results,
                filter_types=list(self.filter_config.keys()),
            )
            secondary_results = secondary_results[filtered_results]

        primary_boxes = np.array(primary_detections.get("xyxyn", []))
        secondary_boxes = np.array(secondary_results.xyxyn)
        secondary_results = secondary_results.to_dict()


        # If there are no primary detections, return secondary detections with severity level set to 2.
        if len(primary_boxes) == 0:
            secondary_results["severity_level"] = [self.X + 1] * len(secondary_results.get("xyxyn", []))
            return secondary_results

        ious = box_iou_batch(secondary_boxes, primary_boxes)  # Shape: [n_secondary, n_primary]
        primary_matches = np.full(len(secondary_boxes), -1)  # -1 signifies no match.

        for sec_idx in range(len(secondary_boxes)):
            matches = np.where(ious[sec_idx] > iou_threshold)[0]
            if matches.size > 0:
                # Choose the primary detection with the highest IoU.
                best_match = matches[np.argmax(ious[sec_idx, matches])]
                primary_matches[sec_idx] = best_match

        # Create a copy of primary detections to update.
        updated_primary = {key: list(value) for key, value in primary_detections.items() if isinstance(value, list)}

        # Loop over each secondary detection.
        for sec_idx, match_idx in enumerate(primary_matches):
            if match_idx != -1:
                # Overwrite the matching primary detection with secondary detection details.
                updated_primary["xyxyn"][match_idx] = secondary_results["xyxyn"][sec_idx]
                updated_primary["class_id"][match_idx] = secondary_results["class_id"][sec_idx]
                if "confidence_score" in primary_detections and "confidence_score" in secondary_results:
                    updated_primary["confidence_score"][match_idx] = secondary_results["confidence_score"][sec_idx]
                if "severity_level" in primary_detections:
                    updated_primary["severity_level"][match_idx] = self.X + 1
                else:
                    updated_primary["severity_level"] = [self.X + 1] * len(primary_detections["xyxyn"])
            else:
                # No match—append secondary detection as a new detection.
                for key in secondary_results:
                    if secondary_results[key] is None:
                        continue
                    if key in updated_primary:
                        updated_primary[key].append(secondary_results[key][sec_idx])
                    else:
                        updated_primary[key] = [secondary_results[key][sec_idx]]
                if "severity_level" in updated_primary:
                    updated_primary["severity_level"].append(self.X + 1)
                else:
                    updated_primary["severity_level"] = [self.X + 1]
                    
        return updated_primary


if __name__ == "__main__":

    import cv2
    from common_utils.annotate.core import Annotator


    colors = [
        (0, 255, 0),
        (0, 165, 255),
        (0, 0, 255),
    ]

    model = BaseModels(
        weights='/media/appuser/rt_cvision/models/WasteImpurityMultiClass_agr_bunker_V3.pt',
        mlflow=False
    )
    image = '/media/appuser/rt_cvision/agr_impurity_detection.v4.yolo/train/images/AGR_gate02_right_2025-04-08_06-19-55_641f2d4a-1c7a-42b4-aaea-3abab372a2eb.jpg.jpg'

    cv_image = cv2.imread(image)

    print(cv_image.shape)
    primary_detections = model.classify_one(
        image=cv_image,
        conf=0.25,
        mode='detect'
    )

    annotator = Annotator(im=cv_image.copy())
    for xyxy in primary_detections.xyxy:
        annotator.box_label(box=xyxy, color=colors[1])

    cv2.imwrite(
        f'/media/appuser/rt_cvision/agr_impurity_detection.v4.yolo/test.jpg',
        annotator.im.data,
    )


    SeverityRefiner = SeverityLevelXDetector(
        model_path="/media/appuser/rt_cvision/models/WasteImpuritySeverityHigh_agr_bunker_V2.pt",
        conf_threshold=0.35,
        X=2,
    )

    primary_detections = primary_detections.to_dict()
    primary_detections['severity_level'] = [int(cls_id) for cls_id in primary_detections.get('class_id', [])]
    updated_primary = SeverityRefiner.integrate(
        primary_detections,
        full_image=cv_image,
    )

    print(updated_primary)
    updated_primary = Detections.from_dict(
        updated_primary
    )

    print(updated_primary)
    annotator = Annotator(im=cv_image.copy())
    for i, xyxy in enumerate(updated_primary.xyxy):
        annotator.box_label(box=xyxy, color=colors[updated_primary.class_id[i]])

    cv2.imwrite(
        f'/media/appuser/rt_cvision/agr_impurity_detection.v4.yolo/test_update.jpg',
        annotator.im.data,
    )
