import pytest
import numpy as np
from common_utils.object_size.zones import Zone, ZoneConfig
from common_utils.object_size.core import ObjectSizeBase


examples_zones = [
    [0.0, 1.0, 0.0, 0.34, 0.0046],
    [0.0, 1.0, 0.34, 1.0, 0.0038],
]

bboxes = np.array([
    [0.37565,     0.54053,     0.56576,     0.68359],
    [0.42318,     0.10693,     0.52604,     0.30518],
    [0.3418,     0.12646,     0.45247,     0.29883],
])

input_shape = (2048, 1536, 3)

zones = []
for zone in examples_zones:
    zones.append(
        Zone(
            zone[0], zone[1], zone[2], zone[3], zone[4]  
        )
    )

def test_object_size_estimation_with_zones():
    zone_config = ZoneConfig(zones)
    estimator = ObjectSizeBase(zone_config=zone_config)

    _, lengths, areas = estimator.compute_object_length_bbox(
        bboxes=bboxes,
        input_shape=input_shape,
        correction_factor=1.0
    )

    assert round(lengths[0] * 100) == 111
    assert round(lengths[1] * 100) == 187
    assert round(lengths[2] * 100) == 162


test_object_size_estimation_with_zones()