"""Recorded RGBD/hand evidence for one apple verification lift. No control.

Post-contact support is deliberately not required: a held apple should leave its
support surface. Identity comes from the fresh confirmed pre-close measurement;
new measured geometry must uniquely follow the palm across the verification lift.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
from apple_rgbd_candidate import estimate
from hand_camera_ray import rotation
from target_anchor_tracking import point_transform
from run_apple_close_lift_trial import lift_allowed
from lift_settled_window import bounds as settled_bounds,contains as settled_contains
from task_outcome_policy import (MIN_VERIFICATION_LIFT_M, MAX_PALM_FOLLOW_ERROR_M,
                                 MAX_HOLD_OPENING_DRIFT_RAD)


def palm_in_base(snapshot):
    b=snapshot['base_from_hand'];p=snapshot['palm_from_hand']
    orient=rotation(b['quaternion'])@rotation(p['quaternion']).T
    return np.asarray(b['translation'])-orient@np.asarray(p['translation'])


def measured(capture):
    observation=json.loads((capture/'observation.json').read_text())
    snapshot=json.loads((capture/'snapshot.json').read_text())
    if observation.get('identity_tracking',{}).get('status')=='near_geometry_tracked':
        from near_apple_geometry import recorded_candidate
        candidate=recorded_candidate(observation,snapshot,
            np.asarray(Image.open(capture/'rgb.png').convert('RGB')),
            np.load(capture/'depth_m.npy',allow_pickle=False),directory=capture.parent)
        return observation,snapshot,candidate
    detections=[d for d in observation['detections'] if d['name']=='apple']
    if len(detections)!=1:raise ValueError('ambiguous_postcontact_apple_geometry')
    candidate=estimate(np.asarray(Image.open(capture/'rgb.png').convert('RGB')),
        np.load(capture/'depth_m.npy',allow_pickle=False),snapshot,detections[0],geometry_proposal_only=True)
    return observation,snapshot,candidate


def pair(before,after,opening):
    bo,bs,bc=before;ao,ass,ac=after
    stamp0=bc['stamp_ns'];stamp1=ac['stamp_ns']
    if bs.get('tracking_epoch')!=ass.get('tracking_epoch') or not bs.get('tracking_epoch'):
        raise ValueError('epoch_mismatch')
    if not 0<(stamp1-stamp0)/1e9<=45:raise ValueError('invalid_verification_capture_order')
    if bo.get('identity_tracking',{}).get('identity_confirmed') is not True:
        raise ValueError('preclose_identity_not_confirmed')
    j0,j1=bs['joints'],ass['joints']
    joint_lift=j1['arm_lift_joint']-j0['arm_lift_joint']
    if not np.isfinite([joint_lift,opening,j1['hand_motor_joint']]).all():
        raise ValueError('invalid_lift_or_opening')
    if not .02<opening<=.62 or not .02<j1['hand_motor_joint']<=.62 or abs(j1['hand_motor_joint']-opening)>MAX_HOLD_OPENING_DRIFT_RAD:
        raise ValueError('holding_opening_not_preserved')
    targets=[];palms=[]
    for snapshot,candidate in ((bs,bc),(ass,ac)):
        stamp=candidate['stamp_ns'];tf=snapshot['tracking_transforms']['odom_from_base_at_depth']
        targets.append(np.asarray(point_transform(candidate['center_m'],tf,stamp=stamp,source='base_link',target='odom')))
        palms.append(np.asarray(point_transform(palm_in_base(snapshot),tf,stamp=stamp,source='base_link',target='odom')))
    object_delta=targets[1]-targets[0];palm_delta=palms[1]-palms[0]
    follow_error=float(np.linalg.norm(object_delta-palm_delta))
    radius_delta=abs(ac['radius_m']-bc['radius_m'])
    if not np.isfinite([*object_delta,*palm_delta,follow_error,radius_delta]).all():
        raise ValueError('invalid_measured_lift_geometry')
    if min(joint_lift,palm_delta[2],object_delta[2]) < MIN_VERIFICATION_LIFT_M:
        raise ValueError('object_did_not_leave_support_with_palm')
    if abs(joint_lift-palm_delta[2]) > .02:
        raise ValueError('measured_palm_and_joint_lift_disagree')
    if follow_error>MAX_PALM_FOLLOW_ERROR_M or radius_delta>.008:raise ValueError('object_palm_following_inconsistent')
    return dict(object_delta_odom_m=object_delta.tolist(),palm_delta_odom_m=palm_delta.tolist(),
        object_palm_follow_error_m=follow_error,apple_center_base_m=ac['center_m'],
        center_minus_palm_odom_m=(targets[1]-palms[1]).tolist(),
        radius_delta_m=radius_delta,stamp_ns=stamp1,
        confidence=next((d['confidence'] for d in ao['detections'] if d['name']=='apple'),None))


def verify(directory):
    trial=json.loads((directory/'result.json').read_text())
    if not lift_allowed(trial.get('close',{})) or trial.get('lift',{}).get('status')!='passed':
        raise ValueError('contact_and_lift_endpoint_required')
    before_path=Path(trial['close']['closure_identity_evidence']['evidence']).parent
    before=measured(before_path);opening=trial['close']['closure_evidence']['observed_opening_rad']
    completed,window_end=settled_bounds(trial);rows=[];rejected=[];transition=[]
    recording=Path(trial.get('observation_directory',directory/'observe'))
    last_complete=trial.get('verification_last_complete_capture')
    if last_complete:
        last_complete=Path(last_complete).resolve()
        if last_complete.parent!=recording.resolve():raise ValueError('last_complete_capture_outside_recording')
    for path in sorted(recording.glob('*/observation.json')):
        if last_complete and path.parent.name>last_complete.name:continue
        snapshot=json.loads((path.parent/'snapshot.json').read_text())
        if snapshot['streams']['depth']['stamp_ns']<=before[2]['stamp_ns']:continue
        if not settled_contains(snapshot,completed,window_end):
            transition.append(dict(capture=path.parent.name,reason='camera_outside_actual_post_endpoint_window'));continue
        try:rows.append(dict(capture=str(path.parent),**pair(before,measured(path.parent),opening)))
        except (ValueError,KeyError) as exc:rejected.append(dict(capture=path.parent.name,reason=str(exc)))
    span=(rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9 if len(rows)>1 else 0
    spread=float(np.max(np.linalg.norm(np.asarray([r['apple_center_base_m'] for r in rows])-rows[-1]['apple_center_base_m'],axis=1))) if rows else None
    relative_spread=(float(np.max(np.linalg.norm(np.asarray([r['center_minus_palm_odom_m'] for r in rows])
                       -rows[-1]['center_minus_palm_odom_m'],axis=1))) if rows else None)
    verified=len(rows)>=3 and span>=.4 and relative_spread<=MAX_PALM_FOLLOW_ERROR_M and not rejected
    return dict(status='recorded_grasp_lift_verified' if verified else 'verification_inconclusive',
        grasp_success_verified=verified,motion_authorized=False,protocol_success_sent=False,
        before_capture=str(before_path),accepted_after_frames=len(rows),after_span_s=span,
        selection='Original image stamps inside actual completed-lift/frozen window; actual object/palm rise and following, not exact10cm or0.5mm joint endpoint',
        lift_process_completed_at_ns=completed,
        verification_window_end_ns=window_end,excluded_transition_captures=transition,
        after_center_spread_m=spread,object_palm_relative_spread_m=relative_spread,rows=rows,rejected=rejected,
        limitation='One simulation trial with RGBD shape and measured contact/hand following. Not success-rate measurement; bridge acquisition-time calibration remains approximate.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    result=verify(a.directory)
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))
