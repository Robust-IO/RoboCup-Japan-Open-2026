"""Fresh measured identity evidence at the publish boundary of one base step.

No motion publisher. Source images must be freshly observed in the same task and
epoch; an old inferred center or a previous successful frame is never a lease.
"""
import json
import math
from pathlib import Path
import time
import numpy as np
from hand_camera_ray import rotation

CONTACT_XY = (.6892952185, .078)
CLOSE_TOLERANCE_M = (.020, .012, .020)


def measured_target(observation, snapshot, *, now_ns, epoch, task_id, current_odom, expected_room='living_room'):
    if expected_room not in ('living_room','bedroom'):raise ValueError('unsupported_reviewed_room')
    if not epoch or not task_id or observation.get('tracking_epoch')!=epoch or snapshot.get('tracking_epoch')!=epoch:
        raise ValueError('tracking_epoch_mismatch')
    identity=observation.get('identity_tracking',{})
    if observation.get('identity_tracking_task_id')!=task_id:
        raise ValueError('tracking_task_mismatch')
    if identity.get('status') not in ('identity_tracked','near_geometry_tracked') or identity.get('identity_confirmed') is not True:
        raise ValueError('current_identity_not_confirmed')
    if observation.get('timing',{}).get('freshness_passed') is not True:
        raise ValueError('vision_result_not_fresh')
    stamp=snapshot['streams']['depth']['stamp_ns'];rgb_stamp=snapshot['streams']['rgb']['stamp_ns']
    age=(now_ns-min(stamp,rgb_stamp))/1e9
    if not 0<=age<=.4 or abs(stamp-rgb_stamp)>30_000_000:
        raise ValueError('vision_evidence_expired_or_unsynchronised')
    measurement=identity['measurement']
    if identity.get('status')=='near_geometry_tracked':
        from near_apple_geometry import check_proof
        check_proof(identity['near_geometry_proof'],snapshot,task_id=task_id,epoch=epoch,measurement=measurement)
    if (measurement.get('stamp_ns')!=stamp or measurement.get('measurement')!='fresh_rgbd'
            or measurement.get('class_name')!='apple' or measurement.get('depth_valid') is not True
            or measurement.get('clipped') is not False):
        raise ValueError('fresh_measured_geometry_required')
    rooms=identity.get('room_evidence',[])
    if not rooms or any(row.get('room_confirmed') is not True or row.get('target_room')!=expected_room for row in rooms):
        raise ValueError('room_evidence_required')
    tf=snapshot['tracking_transforms']['odom_from_base_at_depth']
    if tf.get('stamp_ns')!=stamp or tf.get('source_frame')!='base_link' or tf.get('target_frame')!='odom':
        raise ValueError('tracking_transform_mismatch')
    r=rotation(tf['quaternion']);t=np.asarray(tf['translation'],float)
    point=np.asarray(measurement['point_odom_m'],float)
    numbers=[*t,*point,*current_odom,measurement['radius_m'],measurement['fit_residual_p95_m']]
    if t.shape!=(3,) or point.shape!=(3,) or len(current_odom)!=3 or not np.isfinite(numbers).all():
        raise ValueError('invalid_geometry')
    if not .015<=measurement['radius_m']<=.065 or not 0<=measurement['fit_residual_p95_m']<=.012:
        raise ValueError('unreliable_target_geometry')
    yaw=math.atan2(r[1,0],r[0,0]);angle=math.atan2(math.sin(current_odom[2]-yaw),math.cos(current_odom[2]-yaw))
    if math.dist(t[:2],current_odom[:2])>.01 or abs(angle)>.02:
        raise ValueError('base_moved_since_observation')
    target=r.T@(point-t)
    return dict(stamp_ns=stamp,age_s=age,target_base_m=target.tolist(),
                observation_mode=identity['status'],motion_authorized=False)


def validate(observation, snapshot, *, now_ns, epoch, task_id, current_odom,
             translation, palm_height, contact_xy=CONTACT_XY,max_step_m=.1,min_forward_gap_m=.05,expected_room='living_room',single_approach=False):
    if (type(single_approach) is not bool
            or single_approach and max_step_m is not None
            or not single_approach and max_step_m is None
            or max_step_m is not None and (not math.isfinite(max_step_m) or not .001<=max_step_m<=.25)
            or not math.isfinite(min_forward_gap_m) or not .01<=min_forward_gap_m<=.05):
        raise ValueError('invalid_approach_policy')
    evidence=measured_target(observation,snapshot,now_ns=now_ns,epoch=epoch,task_id=task_id,current_odom=current_odom,expected_room=expected_room)
    target=np.asarray(evidence['target_base_m'])
    if len(translation)!=2 or not np.isfinite([*translation,palm_height,*contact_xy]).all():raise ValueError('invalid_geometry')
    gap=target[:2]-np.asarray(contact_xy)
    step=np.asarray(translation,float)
    remaining=gap-step
    distance=float(np.linalg.norm(step))
    if not distance>0 or max_step_m is not None and distance>max_step_m+1e-7 or step[0]<=0:
        raise ValueError('step_outside_bounded_approach')
    if abs(target[2]-palm_height)>.025:
        raise ValueError('target_height_not_aligned')
    if remaining[0]<min_forward_gap_m or np.dot(gap,step)<=0 or np.linalg.norm(remaining)>=np.linalg.norm(gap):
        raise ValueError('step_overshoots_or_moves_away')
    return dict(evidence,
                remaining_target_minus_contact_xy_m=remaining.tolist(),
                requires='Independent finite endpoint and execution policy checks')


def closure_position_error(target,palm_height):
    """The ORIGINAL close region, shared by scheduling and final lease checks."""
    if not math.isfinite(palm_height):raise ValueError('invalid_palm_height')
    target=np.asarray(target,float)
    if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('invalid_target_position')
    error=target-[*CONTACT_XY,palm_height]
    if np.any(np.abs(error)>CLOSE_TOLERANCE_M):raise ValueError('target_outside_reviewed_closure_region')
    return error


def validate_terminal_correction(observation,snapshot,*,now_ns,epoch,task_id,current_odom,
                                 translation,palm_height,previous,recording,expected_room='bedroom'):
    """A NEW measured corrective action, never replay of a failed command."""
    from grasp_terminal_correction import completed_context, terminal_delta
    completed=completed_context(previous)
    context=dict(recording=str(Path(recording).resolve()),epoch=epoch,task_id=task_id)
    if previous.get('identity_context')!=context:
        raise ValueError('terminal_predecessor_task_mismatch')
    if completed>now_ns:
        raise ValueError('terminal_predecessor_from_future')
    if math.dist(current_odom[:2],previous['final_odom'][:2])>.01 or abs(math.atan2(
            math.sin(current_odom[2]-previous['final_odom'][2]),math.cos(current_odom[2]-previous['final_odom'][2])))>.02:
        raise ValueError('base_moved_after_completed_approach')
    if min(snapshot['streams'][key]['stamp_ns'] for key in ('rgb','depth'))<=completed:
        raise ValueError('post_approach_observation_required')
    evidence=measured_target(observation,snapshot,now_ns=now_ns,epoch=epoch,task_id=task_id,
                             current_odom=current_odom,expected_room=expected_room)
    target=np.asarray(evidence['target_base_m'])
    # Also bounds this explicit mode to the existing horizontal near-contact region.
    terminal_delta(target,palm_height)
    if len(translation)!=2 or not np.isfinite(translation).all():raise ValueError('invalid_terminal_translation')
    step=np.asarray(translation,float);error=target[:2]-CONTACT_XY
    remaining=error-step
    if (np.linalg.norm(step)<=0 or np.dot(error,step)<=0
            or np.linalg.norm(remaining)>=np.linalg.norm(error)
            or np.any(np.abs(remaining)>np.asarray(CLOSE_TOLERANCE_M[:2]))):
        raise ValueError('terminal_correction_not_towards_close_region')
    return dict(evidence,remaining_target_minus_contact_xy_m=remaining.tolist(),
                predecessor_completed_ns=completed,terminal_correction=True,
                requires='Fresh actual closure check AFTER this movement; no closure authorized here')


def validate_closure(observation,snapshot,*,now_ns,epoch,task_id,current_odom,palm_height,expected_room='living_room'):
    evidence=measured_target(observation,snapshot,now_ns=now_ns,epoch=epoch,task_id=task_id,current_odom=current_odom,expected_room=expected_room)
    error=closure_position_error(evidence['target_base_m'],palm_height)
    return dict(evidence,center_minus_contact_xyz_m=error.tolist(),
                grasp_center_verified=False,trial_only=True,requires='Contact and visual lift verification')


def read_latest(directory, *,closure=False, terminal_correction=False, **kwargs):
    from latest_capture_index import latest
    read_started_ns=time.monotonic_ns()
    try:path=latest(directory)
    except FileNotFoundError:raise ValueError('no_current_identity_record') from None
    if closure and terminal_correction:raise ValueError('exclusive_approach_lease_required')
    check=validate_terminal_correction if terminal_correction else validate_closure if closure else validate
    observation=json.loads(path.read_text());snapshot=json.loads((path.parent/'snapshot.json').read_text())
    if observation.get('identity_tracking',{}).get('status')=='near_geometry_tracked':
        from PIL import Image
        from near_apple_geometry import recorded_candidate
        recorded_candidate(observation,snapshot,np.asarray(Image.open(path.parent/'rgb.png').convert('RGB')),
                           np.load(path.parent/'depth_m.npy',allow_pickle=False),directory=path.parent.parent)
        # Charge disk/refit time to the caller's clock; NEVER refresh source stamps.
        kwargs=dict(kwargs,now_ns=kwargs['now_ns']+time.monotonic_ns()-read_started_ns)
    result=check(observation,snapshot,**kwargs)
    return dict(result,evidence=str(path))
