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
    changes=[(0,current[0]+.08)] if stage=='raise' else [(1,0.),(3,-math.pi/2),(0,0.)]
    goal=list(current);steps=[]
    for index,value in changes:
        goal[index]=value;steps.append(dict(kind='arm',goal=goal.copy()))
    return dict(plan,initial=list(current),steps=steps,arm_posture_from_current_feedback=True)


def build(stage,palm_height,holding):
    if stage not in ('prepare','release','raise','retract'):raise ValueError('invalid_stage')
    if not all(math.isfinite(x) for x in (palm_height,holding)) or not .35<=palm_height<=.72 or not .02<=holding<=.62:raise ValueError('unreviewed_height_or_holding')
    lift=palm_height-OFFSET
    horizontal=[lift,FLEX,0.,WRIST,0.,holding]
    if stage=='prepare':initial=[0.,0.,0.,-math.pi/2,0.,holding]
    else:
        initial=horizontal.copy()
        if stage in ('raise','retract'):initial[5]=.65
        if stage=='retract':initial[0]+=.08
    goal=initial.copy();steps=[]
    changes={'prepare':[(0,HIGH_LIFT),(3,WRIST),(1,FLEX),(0,lift)],'release':[(5,.65)],'raise':[(0,lift+.08)],'retract':[(1,0.),(3,-math.pi/2),(0,0.)]}[stage]
    for index,value in changes:
        goal[index]=value;steps.append(dict(kind='hand' if index==5 else 'arm',goal=goal.copy()))
    return dict(initial=initial,steps=steps,palm_height_m=palm_height,no_base_motion=True,requires_outside_table_before_prepare=stage=='prepare',grip_preserved_until_explicit_release=stage!='release',motion_authorized=False)


def release_height(table_height,bottom_below_palm,gap=.015):
    if not all(math.isfinite(x) for x in (table_height,bottom_below_palm,gap)) or not .25<table_height<.75 or not 0<bottom_below_palm<.2 or not .005<=gap<=.03:raise ValueError('invalid_measured_release_geometry')
    height=table_height+bottom_below_palm+gap
    if not .35<=height<=.72:raise ValueError('unreachable_reviewed_release_height')
    return height
