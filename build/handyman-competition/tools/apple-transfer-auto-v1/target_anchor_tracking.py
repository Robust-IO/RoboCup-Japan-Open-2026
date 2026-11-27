"""Short-lived stationary-object anchor; rejects epoch changes and stale anchors."""
import math
import numpy as np
from hand_camera_ray import rotation


def point_transform(point,tf,*,stamp,source,target):
    if tf['stamp_ns']!=stamp or tf['source_frame']!=source or tf['target_frame']!=target:raise ValueError('tf_time_or_frame_mismatch')
    p=np.asarray(point,float);t=np.asarray(tf['translation'],float)
    if p.shape!=(3,) or t.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(t).all():raise ValueError('invalid_transform_point')
    return (rotation(tf['quaternion'])@p+t).tolist()


def seed(candidate,snapshot,*,task_id,identity_confirmed,uncertainty_m):
    if not task_id or identity_confirmed is not True:raise ValueError('identity_not_confirmed')
    if candidate.get('source')!='rgbd' or candidate.get('frame_id')!='base_link' or candidate.get('position_semantics')!='estimated_grasp_center':raise ValueError('invalid_candidate')
    epoch=snapshot.get('tracking_epoch')
    if not epoch:raise ValueError('tracking_epoch_required')
    stamp=candidate['stamp_ns']
    if type(stamp) is not int or stamp<=0 or stamp!=snapshot['streams']['depth']['stamp_ns']:raise ValueError('candidate_capture_mismatch')
    if not math.isfinite(uncertainty_m) or not 0<=uncertainty_m<=.03:raise ValueError('invalid_uncertainty')
    xyz=point_transform(candidate['center_m'],snapshot['tracking_transforms']['odom_from_base_at_depth'],stamp=stamp,source='base_link',target='odom')
    return dict(task_id=task_id,epoch=epoch,class_name='apple',identity_confirmed=True,source='tracked_rgbd',
                point_odom_m=xyz,stamp_ns=stamp,uncertainty_m=uncertainty_m,object_motion_model='stationary_before_contact')


def propagate(anchor,snapshot,*,task_id,contact_started=False,max_age_s=2.):
    if not math.isfinite(max_age_s) or not 0<max_age_s<=5:raise ValueError('invalid_age_policy')
    if anchor['task_id']!=task_id or not task_id:raise ValueError('task_mismatch')
    if anchor.get('identity_confirmed') is not True or anchor.get('source')!='tracked_rgbd' or anchor.get('class_name')!='apple':raise ValueError('invalid_anchor_identity')
    if not snapshot.get('tracking_epoch') or anchor['epoch']!=snapshot['tracking_epoch']:raise ValueError('tracking_epoch_changed')
    if contact_started or anchor.get('object_motion_model')!='stationary_before_contact':raise ValueError('anchor_invalid_after_contact')
    stamp=snapshot['streams']['hand']['stamp_ns'];age=(stamp-anchor['stamp_ns'])/1e9
    if not 0<=age<=max_age_s:raise ValueError('stale_or_reversed_anchor')
    xyz=point_transform(anchor['point_odom_m'],snapshot['tracking_transforms']['base_from_odom_at_hand'],stamp=stamp,source='odom',target='base_link')
    uncertainty=anchor['uncertainty_m']+.002*age
    if not math.isfinite(uncertainty) or not 0<=uncertainty<=.03:raise ValueError('anchor_uncertainty_exceeded')
    return dict(task_id=task_id,class_name='apple',identity_confirmed=True,frame_id='base_link',stamp_ns=stamp,
                source='tracked_rgbd',point_m=xyz,uncertainty_m=uncertainty,motion_authorized=False,
                room_confirmed=False,assumption='object_stationary_until_contact')
