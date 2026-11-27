"""Joint-opening evidence is a contact surrogate, never a pressure reading."""
import math


def grip_retained(opening, closed_opening, tolerance_rad=.03):
    return (math.isfinite(opening) and math.isfinite(closed_opening)
            and abs(opening - closed_opening) < tolerance_rad)


def lift_sample_passes(object_delta, palm_delta, retained, stamp_ns, previous_stamp_ns):
    """Require fresh measured motion, not a commanded lift or cached image."""
    values = list(object_delta) + list(palm_delta)
    if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
        return False
    return bool(retained and stamp_ns > previous_stamp_ns
                and object_delta[2] >= .05
                and math.dist(object_delta, palm_delta) <= .025)
