"""Class-independent HSR tabletop transfer primitives.

Inputs are measured support height and held-object geometry in the *release*
palm orientation. Object names never select heights, travel, or motion steps.
Detection, held-shape fitting and actual placement verification remain adapters.
"""
import math
from numbers import Real

# Calibrated standard horizontal HSR posture; physical lift from Unity HSRCommon.
OFFSET=.2771978151685961
FLEX=-1.7
WRIST=.1292036732051034
LIFT_MIN_M=0.
LIFT_MAX_M=.69
PREFERRED_CLEARANCE_M=.08

def _finite(values):return all(isinstance(v,Real) and not isinstance(v,bool) and math.isfinite(v) for v in values)

def lift_targets(palm_height,current_lift=0.):
    if not _finite((palm_height,current_lift)):raise ValueError('finite_measured_height_required')
    lift=palm_height-OFFSET
    if not -1e-9<=lift<=LIFT_MAX_M+1e-9 or not -1e-6<=current_lift<=LIFT_MAX_M+1e-6:
        raise ValueError('required_height_outside_physical_flat_arm_range')
    lift=min(LIFT_MAX_M,max(LIFT_MIN_M,lift))
    high=min(LIFT_MAX_M,max(current_lift,lift+PREFERRED_CLEARANCE_M))
    return dict(release_lift_m=lift,extension_lift_m=high,palm_height_m=palm_height,
        cleanup_lift_m=min(LIFT_MAX_M,lift+PREFERRED_CLEARANCE_M),
        available_cleanup_raise_m=min(PREFERRED_CLEARANCE_M,LIFT_MAX_M-lift),
        physical_lift_range_m=[LIFT_MIN_M,LIFT_MAX_M],object_class_independent=True,
        height_source='measured_support_plus_actual_held_bottom_offset_and_release_gap')

def release_height(table_height,bottom_below_palm,gap=.015):
    # Signed offset: an object can be held above or below the palm. Feasibility
    # follows real joint travel, not an object-category or furniture-height cap.
    if not _finite((table_height,bottom_below_palm,gap)) or not .005<=gap<=.03:
        raise ValueError('invalid_measured_release_geometry')
    value=table_height+bottom_below_palm+gap
    lift_targets(value)
    return value

def cleanup_lift(current_lift):
    if not _finite((current_lift,)) or not -1e-6<=current_lift<=LIFT_MAX_M+1e-6:
        raise ValueError('invalid_current_lift')
    return min(LIFT_MAX_M,max(0.,current_lift)+PREFERRED_CLEARANCE_M)

def bottom_offset_from_vertices(vertices_palm):
    if vertices_palm is None or len(vertices_palm)==0 or any(len(v)!=3 or not _finite(v) for v in vertices_palm):
        raise ValueError('measured_held_vertices_required')
    return float(-min(v[2] for v in vertices_palm))

def build_motion(stage,palm_height,holding):
    if stage not in ('prepare','release','raise','retract'):raise ValueError('invalid_stage')
    if not _finite((holding,)) or not .02<=holding<=.62:raise ValueError('unreviewed_height_or_holding')
    h=lift_targets(palm_height);lift=h['release_lift_m']
    horizontal=[lift,FLEX,0.,WRIST,0.,holding]
    initial=[0.,0.,0.,-math.pi/2,0.,holding] if stage=='prepare' else horizontal.copy()
    if stage in ('raise','retract'):initial[5]=.65
    if stage=='retract':initial[0]=h['cleanup_lift_m']
    changes={'prepare':[(0,h['extension_lift_m']),(3,WRIST),(1,FLEX),(0,lift)],
        'release':[(5,.65)],'raise':[(0,h['cleanup_lift_m'])],
        'retract':[(1,0.),(3,-math.pi/2),(0,0.)]}[stage]
    goal=initial.copy();steps=[]
    for index,value in changes:
        goal[index]=value;steps.append(dict(kind='hand' if index==5 else 'arm',goal=goal.copy()))
    return dict(initial=initial,steps=steps,palm_height_m=palm_height,height_plan=h,
        no_base_motion=True,requires_outside_table_before_prepare=stage=='prepare',
        grip_preserved_until_explicit_release=stage!='release',motion_authorized=False,
        common_transfer_policy='table_transfer_core_v1')

def _state(runtime):
    joints=list(runtime.state()['joints'])
    if len(joints)!=6 or not _finite(joints):raise ValueError('finite_current_robot_state_required')
    return joints

def prepare_release(runtime,palm_height,holding=None,*,already_extended=False):
    """Raise outside the surface, flatten, then set the measured release height.

    The caller must verify real support, outside-table extension clearance and
    current hold first. This primitive neither opens the hand nor moves the base.
    """
    state=_state(runtime)
    if holding is not None and (not _finite((holding,)) or abs(state[5]-holding)>.08):
        raise ValueError('actual_grip_changed_before_release_preparation')
    h=lift_targets(palm_height,state[0]);runtime.record(dict(event='common_dynamic_table_height_plan',**h))
    if not already_extended:
        target=_state(runtime);target[0]=h['extension_lift_m']
        runtime.move_joints(target,'raise_tucked_before_placement')
        target=_state(runtime);target[1]=FLEX;target[3]=WRIST
        runtime.move_joints(target,'extend_horizontal_above_table')
    target=_state(runtime);target[0]=h['release_lift_m']
    runtime.move_joints(target,'set_measured_release_height')
    return h

def release_open(runtime,before_publish):
    if not callable(before_publish):raise ValueError('live_support_verifier_required')
    def verify():
        result=before_publish()
        if result is False or isinstance(result,dict) and result.get('ready') is False:
            raise ValueError('current_support_not_ready_no_release')
    target=_state(runtime);target[5]=.65
    return runtime.move_joints(target,'release_once',hand=True,before_publish=verify)

def raise_after_release(runtime):
    target=_state(runtime);goal=cleanup_lift(target[0])
    if abs(goal-target[0])<.001:
        runtime.record(dict(event='no_lift_headroom_withdraw_before_folding',actual_lift_m=target[0]))
        return dict(status='already_at_physical_lift_limit',arm_commands_sent=False)
    target[0]=goal
    return runtime.move_joints(target,'raise_open_hand')

def fold_after_withdrawal(runtime):
    target=_state(runtime);target[:5]=[0.,0.,0.,-math.pi/2,0.]
    return runtime.move_joints(target,'fold_and_lower_empty_after_placement')

def extension_retreat(clearance):
    values=[clearance.get('minimum_clearance_m'),clearance.get('clearance_m')]
    if not _finite(values) or values[0]<0:raise ValueError('finite_measured_extension_clearance_required')
    return max(0.,values[0]-values[1]+.02) if not clearance['ready'] else 0.
