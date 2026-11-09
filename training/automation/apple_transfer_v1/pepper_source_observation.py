"""Processed identity evidence while stationary, separate from action RGBD.

The capture was fresh when acquired. Classification/support fitting can finish
later; the current base/arm/head must remain at the observed pose. Every later
approach and close still requires newly measured action-time depth.
"""
import math
import numpy as np
from hand_camera_ray import rotation
from task_outcome_policy import pose_matches


def source_identity_evidence(snapshot,state,current_head,now_ns,epoch):
    if not epoch or snapshot.get('tracking_epoch')!=epoch:raise ValueError('source_identity_epoch_changed')
    stamps=[snapshot['streams'][k]['stamp_ns'] for k in ('rgb','depth')]
    if (any(type(s) is not int or not 0<=now_ns-s<=2_000_000_000 for s in stamps)
            or abs(stamps[0]-stamps[1])>30_000_000):raise ValueError('source_identity_processing_expired')
    if not -20_000_000<=now_ns-state['stamp_ns']<=600_000_000:
        raise ValueError('source_identity_current_feedback_required')
    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    if not pose_matches(state['joints'],[snapshot['joints'][name] for name in names]):
        raise ValueError('arm_moved_during_source_identity_processing')
    old_head=[snapshot['joints'][k] for k in ('head_pan_joint','head_tilt_joint')]
    if len(current_head)!=2 or not np.isfinite(current_head).all() or max(abs(a-b) for a,b in zip(current_head,old_head))>.03:
        raise ValueError('head_moved_during_source_identity_processing')
    tf=snapshot['tracking_transforms']['odom_from_base_at_depth']
    if tf['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']:raise ValueError('identity_image_time_pose_required')
    r=rotation(tf['quaternion']);yaw=math.atan2(r[1,0],r[0,0]);current=state['poses']['odom']
    distance=float(np.linalg.norm(np.asarray(current[:2])-tf['translation'][:2]))
    angle=abs(math.atan2(math.sin(current[2]-yaw),math.cos(current[2]-yaw)))
    if distance>.02 or angle>.03:raise ValueError('base_moved_during_source_identity_processing')
    return dict(identity_processing_evidence_only=True,oldest_image_age_s=(now_ns-min(stamps))/1e9,
        maximum_identity_processing_age_s=2.,current_joint_age_s=(now_ns-state['stamp_ns'])/1e9,
        captured_joint_stamp_ns=snapshot['joint_stamp_ns'],current_joint_stamp_ns=state['stamp_ns'],
        stationary_base_translation_m=distance,stationary_base_yaw_rad=angle,
        original_timestamps_unchanged=True,new_action_time_depth_still_required=True,motion_authorized=False)
