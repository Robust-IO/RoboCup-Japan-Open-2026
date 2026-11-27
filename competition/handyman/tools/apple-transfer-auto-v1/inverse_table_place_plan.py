"""Measured-height tabletop placement, prepared OUTSIDE before base approach.

Pure planning; no publishers. Reuses the verified horizontal arm configuration.
"""
import math
from task_outcome_policy import pose_matches, hand_is_open

OFFSET=.2771978151685961
WRIST=.1292036732051034
FLEX=-1.7
HIGH_LIFT=.4428021848314039


def release_from_current(plan, current):
    """A leased release opens the hand, without resetting arm joints to a plan.

Only call when publication will independently check current object/support RGBD.
This helper is not a substitute for that lease.
"""
    if (len(current)!=6 or not all(type(v) in (int,float) and math.isfinite(v) for v in current)
            or not .02<=current[5]<=.62
            or len(plan['steps'])!=1 or plan['steps'][0]['kind']!='hand'):
        raise ValueError('current_held_release_pose_required')
    goal=list(current);goal[5]=.65
    return dict(plan,initial=list(current),steps=[dict(kind='hand',goal=goal)],
                arm_posture_from_current_feedback=True)


def cleanup_from_current(plan, stage, current):
    """After release, raise/retract from actual lift/opening, without replaying XY."""
    if stage not in ('raise','retract') or len(current)!=6:
        raise ValueError('current_open_cleanup_pose_required')
    expected=list(plan['initial']);expected[0]=current[0];expected[5]=current[5]
    if not pose_matches(current,expected) or not hand_is_open(current[5]):
        raise ValueError('current_open_cleanup_pose_required')
    from table_transfer_core import cleanup_lift
    changes=[(0,cleanup_lift(current[0]))] if stage=='raise' else [(1,0.),(3,-math.pi/2),(0,0.)]
    goal=list(current);steps=[]
    for index,value in changes:
        goal[index]=value;steps.append(dict(kind='arm',goal=goal.copy()))
    return dict(plan,initial=list(current),steps=steps,arm_posture_from_current_feedback=True)


def build(stage,palm_height,holding):
    from table_transfer_core import build_motion
    return build_motion(stage,palm_height,holding)


def release_height(table_height,bottom_below_palm,gap=.015):
    from table_transfer_core import release_height as measured_release_height
    return measured_release_height(table_height,bottom_below_palm,gap)
