"""Only already-retired, colocated ring views may use turn-only execution."""
import math


def same_search_position(previous, current):
    try:
        values = [float(p[k]) for p in (previous['pose'], current['pose']) for k in ('x', 'y', 'yaw')]
        if not all(math.isfinite(v) for v in values):
            return False
        a, b = previous['pose'], current['pose']
        angle = math.atan2(math.sin(b['yaw']-a['yaw']), math.cos(b['yaw']-a['yaw']))
        return math.hypot(b['x']-a['x'], b['y']-a['y']) < 1e-6 and 0 < angle <= math.pi/2
    except (KeyError, TypeError, ValueError):
        return False
