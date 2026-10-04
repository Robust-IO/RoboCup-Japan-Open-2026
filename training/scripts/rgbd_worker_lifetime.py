"""Monotonic lifetime shared by the ROS parent and its local GPU worker."""
import math


def worker_deadline(value, now):
    # Legacy standalone worker retains its 300-second limit.
    if value is None:
        return now + 300.0
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('invalid_worker_deadline')
    # An expired parent deadline must never grant another capture window.
    return value
