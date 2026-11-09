"""Pure geometry shared by preflight planning; never imports a ROS controller."""
import math
import numpy as np


def expected_horizontal_center(palm_height, center_minus_palm):
    a = 1.7
    b = math.pi / 2
    palm_x = .141 + math.cos(a)*.005102036 + math.sin(a)*.34923768 + math.cos(b)*.012 + math.sin(b)*.1405
    values = np.asarray(center_minus_palm, float)
    if values.shape != (3,) or not np.isfinite([palm_height, *values]).all():
        raise ValueError('invalid_held_geometry')
    return np.array([palm_x, .078, palm_height]) + values
