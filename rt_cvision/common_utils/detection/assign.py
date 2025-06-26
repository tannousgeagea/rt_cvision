
import logging
import numpy as np
from typing import Dict, List, Optional, Tuple, cast
from common_utils.detection.utils import box_iou_batch
from common_utils.detection.core import Detections


def assign_detection_to_segments(
    segments: Detections,
    detections: Detections,
    iou_threshold: float = 0.3
):
    """
    For each segmentation contour, assign a detection label if matched by IoU.

    Returns a list of enriched attribute dictionaries per contour.
    """
    try:
        if len(segments.xyxyn) == 0 or len(detections.xyxyn) == 0:
            return np.array([], dtype=int), np.array([], dtype=int), np.ones(len(detections), dtype=bool)


        ious = box_iou_batch(
            segments.xyxyn,
            detections.xyxyn,
        )

        matched_indices = np.where(ious > iou_threshold)
        if matched_indices[0].size == 0:
            return np.array([], dtype=int), np.array([], dtype=int), np.ones(len(detections), dtype=bool)

        segment_idx = matched_indices[0]
        detection_idx = matched_indices[1]
        sorted_indices = np.argsort(-ious[segment_idx, detection_idx]) 

        segment_idx = segment_idx[sorted_indices]
        detection_idx = detection_idx[sorted_indices]

        unique_detections, first_occurrences = np.unique(detection_idx, return_index=True)
        segment_idx = segment_idx[first_occurrences]
        detection_idx = detection_idx[first_occurrences]
        all_indices = np.arange(len(detections))
        unmatched_idx = ~np.isin(all_indices, detection_idx)

        return segment_idx, detection_idx, unmatched_idx
    except Exception as err:
        raise ValueError(f"[Assign detection to segments] Error {err}")

def concat_detections(a: Detections, b: Detections) -> Detections:
    combined_data = {}

    if not len(a) or not len(b):
        return a or b
    
    all_keys = set(a.data.keys()).union(b.data.keys())
    for key in all_keys:
        a_data = a.data.get(key, [])
        b_data = b.data.get(key, [])
        if isinstance(a_data, list) and isinstance(b_data, list):
            combined_data[key] = a_data + b_data
        elif isinstance(a_data, np.ndarray) and isinstance(b_data, np.ndarray):
            combined_data[key] = np.concatenate([a_data, b_data])
        else:
            # fallback for mismatched or missing types
            combined_data[key] = list(a_data) + list(b_data)

    return Detections(
        xyxy=np.concatenate([a.xyxy, b.xyxy]),
        xyxyn=np.concatenate([a.xyxyn, b.xyxyn]),
        xy=np.concatenate([a.xy, b.xy]) if a.xy is not None and b.xy is not None else None,
        xyn=np.concatenate([a.xyn, b.xyn]) if a.xyn is not None and b.xyn is not None else None,
        mask=np.concatenate([a.mask, b.mask]) if a.mask is not None and b.mask is not None else None,
        confidence=np.concatenate([a.confidence, b.confidence]) if a.confidence is not None and b.confidence is not None else None,
        class_id=np.concatenate([a.class_id, b.class_id]) if a.class_id is not None and b.class_id is not None else None,
        tracker_id=np.concatenate([a.tracker_id, b.tracker_id]) if a.tracker_id is not None and b.tracker_id is not None else None,
        object_length=np.concatenate([a.object_length, b.object_length]) if a.object_length is not None and b.object_length is not None else None,
        object_area=np.concatenate([a.object_area, b.object_area]) if a.object_area is not None and b.object_area is not None else None,
        uid=np.concatenate([a.uid, b.uid]) if a.uid is not None and b.uid is not None else None,
        data=combined_data,
    )


def enrich_segments_with_detections(
    segments:Detections,
    detections: Detections,
    iou_threshold: float = 0.45,
    confidence_threshold: float = 0.7,
):
    """
    Assign detection data to matched segments and append high-confidence unmatched detections as new segments.
    Returns an enriched `Detections` object.
    """

    # Add detection attributes to matched segments
    try:
        segment_idx, detection_idx, unmatched_mask = assign_detection_to_segments(
            segments, detections, iou_threshold
        )

        # --- 1. Enrich matched segments with detection attributes ---
        enriched_segments = segments

        if len(detection_idx) > 0:
            for seg_i, det_i in zip(segment_idx, detection_idx):
                for key in ["confidence", "class_id", "object_length", "object_area", "uid"]:
                    segment_value = getattr(enriched_segments, key)
                    detection_value = getattr(detections, key)

                    if segment_value is not None and detection_value is not None:
                        segment_value[seg_i] = detection_value[det_i] or segment_value[seg_i]

        if hasattr(detections, "data") and isinstance(detections.data, dict):
            for key, array in detections.data.items():
                if key not in enriched_segments.data:
                    enriched_segments.data[key] = [None] * len(enriched_segments)

                for seg_i, det_i in zip(segment_idx, detection_idx):
                    if isinstance(enriched_segments.data[key], list):
                        if isinstance(enriched_segments.data[key][seg_i], str):
                            enriched_segments.data[key][seg_i] = array[det_i]
                        if isinstance(enriched_segments.data[key][seg_i], list):
                            enriched_segments.data[key][seg_i] += array[det_i]
                    elif isinstance(enriched_segments.data[key], np.ndarray):
                        enriched_segments.data[key][seg_i] = array[det_i]
                        
        unmatched_detections = cast(Detections, detections[unmatched_mask])
        if unmatched_detections.confidence is not None:
            high_conf_detections = cast(Detections, unmatched_detections[unmatched_detections.confidence > confidence_threshold])
            enriched_segments = concat_detections(enriched_segments, high_conf_detections)
    except Exception as err:
        raise ValueError(f"[Enrich segments] Error {err}")
    return enriched_segments