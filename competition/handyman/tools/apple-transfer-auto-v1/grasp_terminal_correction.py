"""Pure horizontal visual-servo scheduling after one successful full approach.

No ROS, navigation, gripper commands, old-run resume or relaxed closure gates.
Each correction is computed from a distinct post-motion RGBD observation.
"""
import math
import numpy as np
from apple_approach_lease import CONTACT_XY, CLOSE_TOLERANCE_M, closure_position_error

MAX_CORRECTIONS = 5


def terminal_delta(target,palm_height):
    target=np.asarray(target,float)
    if target.shape!=(3,) or not np.isfinite(target).all() or not math.isfinite(palm_height):
        raise ValueError('invalid_terminal_geometry')
    error=target-np.asarray([*CONTACT_XY,palm_height])
    if abs(error[2])>CLOSE_TOLERANCE_M[2]:
        raise ValueError('terminal_height_requires_arm_replanning')
    try:closure_position_error(target,palm_height)
    except ValueError as exc:
        if str(exc)!='target_outside_reviewed_closure_region':raise
    else:return None
    # Reuse the geometry handoff's existing .15m forward / .06m lateral region;
    # this is not a second long approach or a new translation cap for the coarse move.
    if not -.020<=error[0]<=.15 or abs(error[1])>.06:
        raise ValueError('target_outside_terminal_near_region')
    return [float(error[0]),float(error[1]),0.]


def residual(target,palm_height):
    error=np.asarray(target,float)-[*CONTACT_XY,palm_height]
    return float(np.linalg.norm(np.maximum(np.abs(error[:2])-CLOSE_TOLERANCE_M[:2],0)))


def completed_context(previous):
    stamp=previous.get('motion_completed_at_ns')
    pose=previous.get('final_odom')
    if (previous.get('status')!='endpoint_verified' or previous.get('command_sent') is not True
            or previous.get('grip_commands_sent') is not False
            or not (previous.get('single_grasp_approach') is True or previous.get('grasp_terminal_correction') is True)
            or type(stamp) is not int or stamp<=0 or not isinstance(pose,list) or len(pose)!=3
            or not all(type(v) in (int,float) and math.isfinite(v) for v in pose)):
        raise ValueError('successful_completed_approach_required_no_failed_action_retry')
    return stamp


def correct_terminal(observe,move,palm_height,previous):
    """Callbacks do fresh reads and unique actions; never retries a failed move.

    observe(completed_ns) -> (target, context)
    move(delta, context, previous) -> newly completed result
    """
    last_residual=None
    corrections=0
    while True:
        completed=completed_context(previous)
        target,context=observe(completed)
        delta=terminal_delta(target,palm_height)
        if delta is None:
            return dict(corrections=corrections,aligned=True,final_target_base_m=list(target),
                        closure_authorized=False,requires='Independent fresh closure lease')
        remaining=residual(target,palm_height)
        if last_residual is not None and remaining>=last_residual:
            raise RuntimeError('terminal_correction_not_converging')
        if corrections>=MAX_CORRECTIONS:
            raise RuntimeError('terminal_correction_budget_exhausted')
        result=move(delta,context,previous)
        next_completed=completed_context(result)
        if next_completed<=completed:
            raise ValueError('new_completed_correction_required')
        previous=result
        last_residual=remaining
        corrections+=1
