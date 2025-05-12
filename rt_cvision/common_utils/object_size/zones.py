# Re-imports and redefinitions after state reset
import cv2
import random
import numpy as np
from dataclasses import dataclass
from typing import List

@dataclass
class Zone:
    x_start_ratio: float
    x_end_ratio: float
    y_start_ratio: float
    y_end_ratio: float
    correction_factor: float

class ZoneConfig:
    def __init__(self, zones: List[Zone]):
        self.zones = zones

    def get_correction_factor(self, x_pixel: int, y_pixel: int, width: int, height: int, baseline:float=1.) -> float:
        for zone in self.zones:
            x_start = int(zone.x_start_ratio * width)
            x_end = int(zone.x_end_ratio * width)
            y_start = int(zone.y_start_ratio * height)
            y_end = int(zone.y_end_ratio * height)
            if x_start <= x_pixel < x_end and y_start <= y_pixel < y_end:
                return zone.correction_factor
        return baseline


def generate_random_color(bgr=True):
    min, max = 0, 255
    red = random.randint(min, max) 
    green = random.randint(min, max) 
    blue = random.randint(min, max) 
    return (blue, green, red) if bgr else (red, green, blue)

def draw_zones_on_image(image: np.ndarray, zone_config: ZoneConfig) -> np.ndarray:
    """
    Draws the configured zones on the image and annotates each zone with its correction factor.

    Args:
        image (np.ndarray): The image on which to draw the zones.
        zone_config (ZoneConfig): The zone configuration with boundaries and correction factors.

    Returns:
        np.ndarray: Image with zones drawn and annotated.
    """
    output_image = image.copy()
    height, width = image.shape[:2]

    for zone in zone_config.zones:
        x1 = int(zone.x_start_ratio * width)
        x2 = int(zone.x_end_ratio * width)
        y1 = int(zone.y_start_ratio * height)
        y2 = int(zone.y_end_ratio * height)

        # Draw rectangle
        cv2.rectangle(output_image, (x1, y1), (x2, y2), color=generate_random_color(), thickness=3)

        # Prepare annotation text
        text = f"{zone.correction_factor:.2f}"
        font_scale = 0.5
        thickness = 1
        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)

        # Position the text at the top-left of the zone
        text_x = x1 + 5
        text_y = y1 + text_size[1] + 5
        cv2.putText(output_image, text, (text_x, text_y),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=font_scale, color=(0, 0, 0), thickness=thickness,
                    lineType=cv2.LINE_AA)

    return output_image
