

import time
import pytz
import uuid
import hashlib
import numpy as np
import logging
import torch
from copy import deepcopy
from datetime import datetime
from typing import Dict, List, Optional, cast, Tuple
from common_utils.ml_models import load_inference_module
from common_utils.detection.core import Detections
from common_utils.filters.core import FilterEngine
from common_utils.trackers import SORTTracker
from common_utils.log import Logger

device = "gpu" if torch.cuda.is_available() else "cpu"

def generate_unique_id():
    # Generate UUID and hash it to a short 8-digit number
    unique_id = int(hashlib.sha256(uuid.uuid4().bytes).hexdigest(), 16) % 100000000
    return unique_id

def is_within_active_time(start_time, end_time, timezone):
    """Check if current local time is within active time window."""
    try:
        local_tz = pytz.timezone(timezone)
        current_time = datetime.now(local_tz).strftime("%H:%M")
        return start_time <= current_time <= end_time
    except Exception as e:
        logging.error(f"Error checking active time window: {e}")
        return False

class Segmentation:
    def __init__(self, config:Dict) -> None:
        assert "type" in config, f"unknown type of model"
        assert "framework" in config, f"unknown framework of model"

        self.map_tracker_id_2_object_uid = {}
        self.config = config
        self.activate_tracking = config.get("activate-tracking", False)
        self.model_cls = load_inference_module(
            config=config
        )

        self.model = self.model_cls(
            weights=config["weights"],
            device=config.get("device") or device,
            config=config,
        )

        self.tracker = SORTTracker()
        self.filter_engine = FilterEngine()
        self.logger = Logger(name="Segmentation Model", level=logging.DEBUG)
        self.filter_config = self.config.get("filter_config")
        if self.filter_config:
            for obj_type, filter_model in self.filter_config.items():
                self.filter_engine.add_model(
                    object_type=obj_type,
                    detection_model=filter_model["model_path"],
                    config=self.filter_config[obj_type],
                    device=config.get("device") or device,
                    mlflow=filter_model["mlflow"]
                )

    def infer(self, image: np.ndarray, confidence_threshold:Optional[float] = 0.25):
        detections = self.model.predict(image, confidence_threshold)
        if self.activate_tracking:
            detections = self.tracker.update(detections)
        
        return detections
        
    def crop_image(self, image:np.ndarray, roi:Optional[List]):
        if not roi:
            return image, (0, 0, image.shape[1], image.shape[0]), roi
        try:
            c_image = image.copy()
            h, w, _ = c_image.shape
            roi_pixels = [(int(x * w), int(y * h)) for x, y in roi]
            x_coords, y_coords = zip(*roi_pixels)
            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)
            return c_image[y_min:y_max, x_min:x_max], (x_min, y_min, x_max, x_max), roi_pixels
        except Exception as err:
            self.logger.error(f"Error in cropping image: {err}")
        
        return image, (0, 0, image.shape[1], image.shape[0]), roi

    def filter_detections(self, image:np.ndarray, detections:Detections):
        if not self.filter_config:
            return detections, []
        
        filtered_results, unwanted_rois = self.filter_engine.filter_objects(
            image=image,
            segmentation_results=detections,
            filter_types=list(self.filter_config.keys()),
        )
        self.logger.info(f"Filtered: {filtered_results.shape}")
        if filtered_results.shape[0] == 0:
            return detections, []
                    
        return cast(Detections, detections[filtered_results]), unwanted_rois

    def register(self, detections: Detections) -> Tuple[Detections, Detections]:
        """
        Registers tracker IDs with persistent UUIDs.

        - Assigns a unique UID to each detection.
        - If a detection's tracker_id has already been seen, reuses its UID.
        - Returns a new Detections object containing only detections with newly registered tracker_ids.
        - Does not mutate the input `detections` object.

        Args:
            detections (Detections): Incoming detections with tracker_id.

        Returns:
            Detections: Subset of detections that were newly registered.
        """

        try:
            n = len(detections)
            if detections.tracker_id is None:
                tracker_ids = np.array([generate_unique_id() for _ in range(n)])
                detections.tracker_id = tracker_ids
            else:
                tracker_ids = detections.tracker_id

            uids = np.array([str(uuid.uuid4()) for _ in range(n)])
            new_indices = []

            for i, tid in enumerate(tracker_ids):
                if tid not in self.map_tracker_id_2_object_uid:
                    self.map_tracker_id_2_object_uid[tid] = uids[i]
                    new_indices.append(i)
                else:
                    uids[i] = self.map_tracker_id_2_object_uid[tid]

            # Create updated detection set with new uids
            updated_detections = Detections(
                xyxy=detections.xyxy,
                xyxyn=detections.xyxyn,
                xy=detections.xy,
                xyn=detections.xyn,
                mask=detections.mask,
                confidence=detections.confidence,
                class_id=detections.class_id,
                tracker_id=detections.tracker_id,
                object_length=detections.object_length,
                object_area=detections.object_area,
                uid=uids,
                data=detections.data,
            )

            return detections, cast(Detections, updated_detections[new_indices])

        except Exception as err:
            self.logger.error(f"Unexpected error during detection registration: {err}")
            raise err
    
    def classify(self, detections:Detections):
        object_length_threshold = self.config['object-length-thresholds']
        if not object_length_threshold:
            return detections
        long_threshold = object_length_threshold[-1]['min']
        object_lengths = np.array(detections.object_length)
        is_long = object_lengths >= long_threshold
        attributes = [["long"] if val else ["short"] for val in is_long]
        detections.data["attributes"] = attributes
        return detections


    def run(self, image:np.ndarray, confidence_threshold:Optional[float] = 0.25):
        try:
            if not is_within_active_time(
                start_time=self.config.get('start_time', '00:00'),
                end_time=self.config.get('end_time', '23:59'),
                timezone=self.config.get('timezone', 'UTC')
            ):
                self.logger.info(f"Segmentation is inactive outside of {self.config.get('start_time', '00:00')} - {self.config.get('end_time', '23:59')}")
                return Detections.from_dict({}), [], None
            
            assert not image is None, f'Image is None'
            assert not self.model is None, f'Model is None'
            start_time = time.time()
            c_image, (xmin, ymin, _, _), roi = self.crop_image(image, roi=self.config.get('roi'))
            detections = self.infer(image=c_image, confidence_threshold=confidence_threshold)
            
            start_inf = time.time()
            self.logger.info(f"3. Total Inference Time: {round((time.time() - start_inf) * 1000, 2)} ms")

            if roi:
                detections = detections.adjust_to_roi(
                    offset=(xmin, ymin),
                    crop_size=c_image.shape[:2],
                    original_size=image.shape[:2],
                )
            detections, unwanted_rois = self.filter_detections(image, detections)
            self.logger.info(f"4. Total Prediction Time: {round((time.time() - start_time) * 1000, 2)} ms")

            torch.cuda.empty_cache()
            max_memory_usage = torch.cuda.max_memory_allocated() / (1024 * 1024)
            reserved_memory = torch.cuda.max_memory_reserved() / (1024 * 1024)

            self.logger.info(f'Memory Usage: {max_memory_usage} mb')
            self.logger.info(f'Reserved Memory Usage: {reserved_memory} mb')

            return detections, unwanted_rois, roi
        except Exception as err:
            raise ValueError(f'Unexpected Error in Segmentation: {err}')
        