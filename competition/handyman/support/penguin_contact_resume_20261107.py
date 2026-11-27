"""Resume one verified pre-lift contact without another close command."""
import copy,json,math
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from task_outcome_policy import pose_matches
from penguin_tasks import validate_predecessor_task

def complete_recorded_hand_inverse(snapshot):
    s=copy.deepcopy(snapshot);tracking=s['tracking_transforms']
    if 'base_from_odom_at_hand' in tracking:return s
    if (s.get('transform_basis')!='stationary_transform_snapshot'
            or s.get('stop_observe_act',{}).get('captured_after_stop') is not True):
        raise ValueError('recorded_stationary_capture_required')
    stamp=s['streams']['hand']['stamp_ns'];depth=s['streams']['depth']['stamp_ns']
    tf=tracking['odom_from_base_at_depth']
    if (abs(stamp-depth)>100_000_000 or tf['stamp_ns']!=depth
            or tf['source_frame']!='base_link' or tf['target_frame']!='odom'):
        raise ValueError('same_stopped_capture_transforms_required')
    r=Rotation.from_quat(tf['quaternion']);inverse=r.inv()
    tracking['base_from_odom_at_hand']=dict(translation=(-inverse.apply(tf['translation'])).tolist(),
        quaternion=inverse.as_quat().tolist(),stamp_ns=stamp,source_frame='odom',target_frame='base_link',
        basis='inverse_of_recorded_stationary_base_transform',derived_from_depth_stamp_ns=depth,
        actual_transform_stamp_ns=tf.get('actual_transform_stamp_ns'))
    s['recorded_hand_inverse_completed_without_changing_pixels_or_timestamps']=True
    return s

def validate_contact(previous,metadata,current,boot_id):
    if (previous.get('status')!='stopped_requires_actual_state_review'
            or previous.get('class_name')!='toy_penguin' or previous.get('stage')!='close_once'
            or previous.get('reason')!="'base_from_odom_at_hand'"
            or previous.get('close_commands_sent')!=1 or previous.get('release_commands_sent')!=0
            or previous.get('close',{}).get('status')!='passed' or metadata['boot_id']!=boot_id):
        raise ValueError('specific_unlifted_contact_checkpoint_required')
    old=previous['last_fresh_robot_state']
    if not pose_matches(current['joints'],old['joints']):raise ValueError('contact_arm_or_grip_changed')
    if math.dist(current['poses']['odom'][:2],old['poses']['odom'][:2])>.02:
        raise ValueError('contact_base_moved_since_checkpoint')
    angle=current['poses']['odom'][2]-old['poses']['odom'][2]
    if abs(math.atan2(math.sin(angle),math.cos(angle)))>.03:raise ValueError('contact_base_rotated')
    return True

def load_resume(predecessor,cfg,boot_id,runtime_type,root,record):
    import rclpy
    p=Path(predecessor);old=json.loads((p/'result.json').read_text());meta=json.loads((p/'run-meta.json').read_text())
    validate_predecessor_task(cfg,json.loads((p/'config.snapshot.json').read_text()))
    capture=Path(old['close']['before_capture'])
    if not capture.resolve().is_relative_to(p.resolve()):raise ValueError('preclose_capture_outside_original_task')
    snapshot=complete_recorded_hand_inverse(json.loads((capture/'snapshot.json').read_text()))
    model=json.loads((p/'grasp/source-reference.json').read_text())
    before=json.loads((capture/'object-measurement.json').read_text())
    if before.get('current_head_depth_measured') is not True or before['stamp_ns']!=snapshot['streams']['depth']['stamp_ns']:
        raise ValueError('actual_preclose_penguin_depth_required')
    rclpy.init();runtime=runtime_type(root,snapshot['tracking_epoch'],record)
    try:
        runtime.sample();current=runtime.state();runtime.ownership();validate_contact(old,meta,current,boot_id)
    except BaseException:
        runtime.shutdown();rclpy.shutdown();raise
    sample=dict(snapshot=snapshot,rgb=cv2.cvtColor(cv2.imread(str(capture/'rgb.png')),cv2.COLOR_BGR2RGB),
        depth=np.load(capture/'depth_m.npy'),hand=cv2.imread(str(capture/'hand.png')))
    return dict(runtime=runtime,model=model,before=before,before_sample=sample,close=old['close'],
        holding=current['joints'][5],state=current,source_observations=old['source_observations'])
