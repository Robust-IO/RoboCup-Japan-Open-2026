"""Phase-5 offline handoff contract, not a collision or reachability checker.

Producer must supply actual RGB-D evidence, not Unity object coordinates.
All times must use the same ROS clock. A clock reset requires new evidence.
No publishers, robot movement, or protocol success messages are generated.
"""
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class TargetEvidence:
    task_id: str
    class_name: str
    frame_id: str
    stamp: float
    point: tuple
    measured_depth_m: float
    depth_valid: bool
    room_confirmed: bool


def check_target(evidence, *, task_id, class_name, now, max_age=0.5):
    """Return (eligible_for_planning, reason); never authorize physical motion."""
    if not isfinite(now) or not isfinite(max_age) or max_age <= 0:
        return False, 'invalid_clock_policy'
    if not task_id or evidence.task_id != task_id:
        return False, 'task_mismatch'
    if not class_name or evidence.class_name != class_name:
        return False, 'class_mismatch'
    if evidence.frame_id != 'base_link':
        return False, 'requires_image_time_transform_to_base_link'
    if not isfinite(evidence.stamp) or not 0 <= now-evidence.stamp <= max_age:
        return False, 'stale_or_future_evidence'
    if len(evidence.point) != 3 or not all(isfinite(v) for v in evidence.point):
        return False, 'invalid_point'
    if not evidence.depth_valid or not isfinite(evidence.measured_depth_m) or evidence.measured_depth_m <= 0:
        return False, 'no_measured_depth'
    if not evidence.room_confirmed:
        return False, 'room_not_confirmed'
    return True, 'planning_only_requires_reachability_collision_and_fresh_joint_checks'
