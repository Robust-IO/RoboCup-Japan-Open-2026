"""Can held-state geometry across retraction/navigation. No ROS or commands."""
import math
import numpy as np

from canned_grasp_geometry import measure_near
from task_outcome_policy import carry_start_from_current,pose_matches


def current_images_and_feedback(snapshot,state,now_ns,epoch,*,joint_max_age_ns=400_000_000):
    """Keep image-time geometry; acquire NEW feedback after processing.

    Captured joint metadata is never retimestamped. It is compared with the
    separately recorded current state, not misrepresented as fresh feedback.
    """
    if type(joint_max_age_ns) is not int or joint_max_age_ns not in (400_000_000,600_000_000):
        raise ValueError('invalid_joint_age_limit')
    if not epoch or snapshot.get('tracking_epoch')!=epoch:raise ValueError('carry_epoch_mismatch')
    stamps=[snapshot['streams'][key]['stamp_ns'] for key in ('rgb','depth')]
    if (any(type(s) is not int or not 0<=now_ns-s<=400_000_000 for s in stamps)
            or abs(stamps[0]-stamps[1])>30_000_000):raise ValueError('fresh_carry_images_required')
    current_stamp=state['stamp_ns']
    if type(current_stamp) is not int or not -20_000_000<=now_ns-current_stamp<=joint_max_age_ns:
        raise ValueError('fresh_current_carry_joint_feedback_required')
    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    captured=[float(snapshot['joints'][key]) for key in names]
    if not pose_matches(state['joints'],captured):raise ValueError('arm_changed_since_carry_image')
    return dict(oldest_image_age_s=(now_ns-min(stamps))/1e9,current_joint_age_s=(now_ns-current_stamp)/1e9,
        image_stamps_ns=stamps,current_joint_stamp_ns=current_stamp,
        captured_joint_stamp_ns=snapshot['joint_stamp_ns'],original_snapshot_unchanged=True,
        captured_joint_sample_used_as_fresh_feedback=False,motion_authorized=False)


def current_held(depth,snapshot,reference,*,rgb=None,appearance=None):
    if reference.get('can_grasp_and_lift_verified') is not True:
        raise ValueError('verified_can_grasp_predecessor_required')
    offset=np.asarray(reference['held_center_minus_palm_base_m'],float)
    palm=np.asarray(snapshot['palm_base_at_depth'],float)
    opening=snapshot['joints']['hand_motor_joint'];holding=reference['held_opening_rad']
    if (offset.shape!=(3,) or palm.shape!=(3,) or not np.isfinite([*offset,*palm,opening,holding]).all()
            or not .02<opening<=.62 or abs(opening-holding)>.03):
        raise ValueError('can_current_holding_feedback_required')
    measured=measure_near(depth,snapshot,palm+offset,reference['can_footprint_radius_m'],reference['can_height_m'],rgb=rgb,appearance=appearance)
    actual=np.asarray(measured['center_m'])-palm
    error=float(np.linalg.norm(actual-offset))
    if error>.02:raise ValueError('can_not_at_measured_hand_offset')
    return dict(measurement=measured,center_minus_palm_base_m=actual.tolist(),
        relative_to_reference_error_m=error,opening_rad=opening,
        current_depth_surface_verified=True,grip_feedback_alone_is_not_proof=True,
        source_identity='same_verified_grasp_predecessor_current_depth_association',motion_authorized=False)


def carry_initial(joints,contact_start_lift,holding):
    if (not math.isfinite(contact_start_lift) or not .24<=contact_start_lift<=.30
            or not math.isfinite(holding) or not .02<holding<=.62):
        raise ValueError('invalid_can_carry_context')
    expected=[contact_start_lift+.1,-1.7,0.,.1292036732051034,0.,holding]
    return carry_start_from_current(expected,list(joints),contact_start_lift)


def tucked_holding(joints,holding):
    return pose_matches(list(joints),[0.,0.,0.,-math.pi/2,0.,holding])


def compare_held(before,after):
    if not before.get('current_depth_surface_verified') or not after.get('current_depth_surface_verified'):
        raise ValueError('actual_can_surfaces_required')
    offset_error=float(np.linalg.norm(np.asarray(after['center_minus_palm_base_m'])-before['center_minus_palm_base_m']))
    radius_change=abs(after['measurement']['footprint_radius_m']-before['measurement']['footprint_radius_m'])
    if offset_error>.02 or radius_change>.008 or abs(after['opening_rad']-before['opening_rad'])>.03:
        raise ValueError('can_hold_changed_during_carry')
    return dict(object_palm_offset_change_m=offset_error,observed_radius_change_m=radius_change,
        grip_opening_change_rad=abs(after['opening_rad']-before['opening_rad']),
        can_still_held_verified=True,placement_verified=False,motion_authorized=False)
