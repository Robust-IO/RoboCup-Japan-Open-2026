"""Functional acceptance for the reviewed transfer, not historical pose replay.

These are measurement allowances, not changes to navigation or mechanical limits.
Current identity, RGBD freshness, table ownership and observed edge checks remain
with their consumers. A planned point/posture is guidance, not proof of success.
"""
import math

MIN_VERIFICATION_LIFT_M = .03
MAX_PALM_FOLLOW_ERROR_M = .02
MAX_HOLD_OPENING_DRIFT_RAD = .03
RELEASE_BOTTOM_GAP_M = (-.01, .05)
PLACED_BOTTOM_GAP_M = .03
PLACED_CENTER_SPREAD_M = .02
OPEN_HAND_MIN_RAD = .60


def pose_matches(actual, expected, *, endpoint=False):
    """1cm/0.03rad start envelope; 5mm/0.02rad commanded endpoint envelope."""
    tolerances = (.005, .02, .02, .02, .02, .02) if endpoint else (.01, .03, .03, .03, .03, .03)
    return (len(actual) == len(expected) == 6
            and all(type(v) in (int, float) and math.isfinite(v) for v in (*actual, *expected))
            and all(abs(a-b) <= t for a, b, t in zip(actual, expected, tolerances)))


def hand_is_open(value):
    return type(value) in (int, float) and math.isfinite(value) and OPEN_HAND_MIN_RAD <= value <= .70


def arm_base_stability(anchor, actual, *, carrying=False):
    """A held-object fold is base-relative, unlike alignment at a support.

    Permit five degrees of passive yaw only during the explicitly selected
    held carry fold. Translation, all other arm stages and command ownership
    retain their existing checks. This is not permission to command the base.
    """
    if (type(carrying) is not bool or len(anchor) != 3 or len(actual) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in (*anchor, *actual))):
        raise ValueError('finite_base_poses_required')
    translation = math.dist(actual[:2], anchor[:2])
    yaw = abs(math.atan2(math.sin(actual[2]-anchor[2]), math.cos(actual[2]-anchor[2])))
    yaw_limit = math.radians(5.) if carrying else .03
    return dict(allowed=translation <= .015 and yaw <= yaw_limit,
                translation_m=translation, yaw_rad=yaw,
                translation_limit_m=.015, yaw_limit_rad=yaw_limit,
                carrying=carrying, anchor=list(anchor), actual=list(actual))


def carry_lower_from_current(actual, holding_opening):
    """Explicit fresh-state completion of an already folded held arm.

    Never re-extend/re-fold it, open the hand, or infer a grasp from posture.
    The caller must separately establish that the same object is still held.
    """
    if len(actual) != 6 or type(holding_opening) not in (int, float) or not math.isfinite(holding_opening):
        raise ValueError('current_folded_holding_pose_required')
    expected = [actual[0], 0., 0., -math.pi/2, 0., holding_opening]
    if (not pose_matches(actual, expected) or not .02 < holding_opening <= .62
            or not 0. <= actual[0] <= .45):
        raise ValueError('current_folded_holding_pose_required')
    return list(actual)


def carry_start_from_current(expected, actual, contact_start_lift):
    """Carry the verified held object from its actual lift, not start+exact10cm."""
    if len(actual)!=6 or len(expected)!=6:
        raise ValueError('current_lifted_holding_pose_required')
    adjusted=list(expected);adjusted[0]=actual[0]
    if (not math.isfinite(contact_start_lift) or not pose_matches(actual,adjusted)
            or not .02<actual[5]<=.62 or actual[0]-contact_start_lift<MIN_VERIFICATION_LIFT_M):
        raise ValueError('current_lifted_holding_pose_required')
    return list(actual)


def robot_ready(joints):
    """Readiness is separate from whether the object has been placed."""
    names = ('arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint', 'hand_motor_joint')
    if any(k not in joints for k in names):
        return False
    values = [joints[k] for k in names]
    return hand_is_open(values[5]) and pose_matches(values[:5]+[.65], [0., 0., 0., -math.pi/2, 0., .65])
