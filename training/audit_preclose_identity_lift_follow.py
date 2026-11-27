"""Independent RECORDED lift audit, not detector output or a live action lease.

Requires actual closure-time identity/contact and a measured lift. If the
post-contact detector misses, independently fits one observed red RGBD surface
near the pre-close center transported by MEASURED palm movement. This prediction
only bounds an association query; pixels/depth must supply every new center.
Canonical detector-only trial results are never overwritten or auto-resumed.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from apple_roi_confirmation import identity_evidence
from hand_camera_ray import rotation
from run_apple_close_lift_trial import lift_allowed
from tracked_apple_surface import measure as fit_surface_geometry
from verify_apple_lift_sequence import measured,pair,palm_in_base
from lift_settled_window import bounds as settled_bounds,contains as settled_contains
from task_outcome_policy import MAX_PALM_FOLLOW_ERROR_M


def associate(rgb,depth,snapshot,before):
    observation,original,center=before
    if observation.get('identity_tracking',{}).get('identity_confirmed') is not True:
        raise ValueError('confirmed_preclose_identity_required')
    if (snapshot.get('tracking_epoch')!=original.get('tracking_epoch')
            or not original.get('tracking_epoch')):raise ValueError('epoch_mismatch')
    if rgb.shape!=(*depth.shape,3) or depth.ndim!=2:raise ValueError('shape_mismatch')
    if abs(snapshot['streams']['rgb']['stamp_ns']-snapshot['streams']['depth']['stamp_ns'])>30_000_000:
        raise ValueError('rgb_depth_unsynchronised')
    # This is a verification hypothesis, NOT an assumption that grasp succeeded.
    expected=palm_in_base(snapshot)+(np.asarray(center['center_m'])-palm_in_base(original))
    k=np.asarray(snapshot['rgb_k']).reshape(3,3)
    if not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0:raise ValueError('invalid_intrinsics')
    v,u=np.indices(depth.shape);tf=snapshot['base_from_rgb'];z=depth
    points=np.stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z),-1)
    points=points@rotation(tf['quaternion']).T+tf['translation']
    r,g,b=rgb.astype(float).transpose(2,0,1)
    mask=(np.isfinite(z)&(z>0)&(z<3)&(r>80)&(r>1.35*g)&(r>1.2*b)&(b>.9*g)
          &(np.linalg.norm(points-expected,axis=2)<.09))
    _,_,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),connectivity=8)
    # Two separately visible substantial surfaces must not be averaged into one.
    if np.count_nonzero(stats[1:,cv2.CC_STAT_AREA]>=80)!=1:
        raise ValueError('unique_verification_rgbd_surface_required')
    # Only use the numerical fit, never its legacy "previously held" method label.
    fit=fit_surface_geometry(rgb,depth,snapshot,expected)
    return dict(center_m=fit['center_base'],radius_m=fit['radius_m'],stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        fit_residual_p95_m=fit['residual_p95_m'],points=fit['pixels'],association_error_m=fit['association_error_m'],
        frame_id='base_link',source='fresh_rgbd_verification_association',geometry_proposal_only=True,
        identity_confirmed=False,detector_confidence=None,motion_authorized=False)


def audit(directory, *, expected_recording=None, expected_epoch=None, expected_task_id=None):
    directory=Path(directory);trial=json.loads((directory/'result.json').read_text())
    if not lift_allowed(trial.get('close',{})) or trial.get('lift',{}).get('status')!='passed':
        raise ValueError('actual_contact_and_lift_endpoint_required')
    recording=Path(trial['observation_directory']).resolve()
    if expected_recording is None:
        if recording!= (directory/'observe').resolve():raise ValueError('owned_recording_required')
    elif (not expected_epoch or not expected_task_id or trial.get('observer_owned') is not False
            or recording!=Path(expected_recording).resolve()):
        raise ValueError('explicit_parent_owned_recording_binding_required')
    if (expected_epoch is None)!=(expected_task_id is None):raise ValueError('task_and_epoch_binding_required')
    if expected_epoch is not None and (trial.get('tracking_epoch')!=expected_epoch
            or trial.get('tracking_task_id')!=expected_task_id):raise ValueError('trial_task_or_epoch_mismatch')
    last_complete=trial.get('verification_last_complete_capture')
    if last_complete:
        last_complete=Path(last_complete).resolve()
        if last_complete.parent!=recording:raise ValueError('last_complete_capture_outside_recording')
    lease=trial['close']['closure_identity_evidence'];before_path=Path(lease['evidence']).parent.resolve()
    if not before_path.is_relative_to(recording):raise ValueError('preclose_evidence_outside_recording')
    before=measured(before_path)
    if expected_epoch is not None and (before[0].get('tracking_epoch')!=expected_epoch
            or before[1].get('tracking_epoch')!=expected_epoch
            or before[0].get('identity_tracking_task_id')!=expected_task_id):
        raise ValueError('preclose_task_or_epoch_mismatch')
    if lease['stamp_ns']!=before[2]['stamp_ns'] or not 0<=lease['age_s']<=.4:
        raise ValueError('actual_publication_preclose_lease_required')
    used_roi=before[0].get('identity_tracking',{}).get('measurement',{}).get('confidence_basis')=='same_frame_near_grasp_roi'
    geometry_preclose=before[0].get('identity_tracking',{}).get('status')=='near_geometry_tracked'
    # measured() has re-fitted current pixels and rechecked all3 original seed files.
    # A geometry-only closure deliberately has no CURRENT YOLO/ROI classification.
    if not geometry_preclose and (trial.get('owned_near_grasp_roi_enabled') is True or used_roi) and identity_evidence(before[0],before[1]) is None:
        raise ValueError('bound_preclose_roi_identity_required')
    if before[0].get('identity_tracking',{}).get('identity_confirmed') is not True:
        raise ValueError('confirmed_preclose_identity_required')
    opening=trial['close']['closure_evidence']['observed_opening_rad']
    completed,window_end=settled_bounds(trial)
    rows=[];rejected=[];transition=[]
    for path in sorted(recording.glob('*/observation.json')):
        if last_complete and path.parent.name>last_complete.name:continue
        snapshot=json.loads((path.parent/'snapshot.json').read_text())
        if (snapshot['streams']['depth']['stamp_ns']<=before[2]['stamp_ns']
                or snapshot['streams']['depth']['stamp_ns']>window_end):continue
        if not settled_contains(snapshot,completed,window_end):
            transition.append(dict(capture=path.parent.name,reason='camera_outside_actual_post_endpoint_window'));continue
        try:
            observation=json.loads(path.read_text())
            if expected_epoch is not None and (observation.get('tracking_epoch')!=expected_epoch
                    or snapshot.get('tracking_epoch')!=expected_epoch
                    or observation.get('identity_tracking_task_id')!=expected_task_id):
                raise ValueError('postlift_task_or_epoch_mismatch')
            apples=[d for d in observation['detections'] if d['name']=='apple']
            if len(apples)>1:raise ValueError('ambiguous_postcontact_apple_geometry')
            if apples:after=measured(path.parent);method='original_detector_box_rgbd'
            else:
                candidate=associate(np.asarray(Image.open(path.parent/'rgb.png').convert('RGB')),
                    np.load(path.parent/'depth_m.npy',allow_pickle=False),snapshot,before)
                after=observation,snapshot,candidate;method='confirmed_preclose_identity_with_unique_current_rgbd_surface'
            value=pair(before,after,opening)
            rows.append(dict(capture=str(path.parent),**value,measurement_method=method,
                             center_measurement=after[2]))
        except (ValueError,KeyError) as exc:rejected.append(dict(capture=path.parent.name,reason=str(exc)))
    span=(rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9 if len(rows)>1 else 0
    spread=float(np.max(np.linalg.norm(np.asarray([r['apple_center_base_m'] for r in rows])-rows[-1]['apple_center_base_m'],axis=1))) if rows else None
    relative_spread=(float(np.max(np.linalg.norm(np.asarray([r['center_minus_palm_odom_m'] for r in rows])
                       -rows[-1]['center_minus_palm_odom_m'],axis=1))) if rows else None)
    verified=len(rows)>=3 and span>=.4 and relative_spread<=MAX_PALM_FOLLOW_ERROR_M and not rejected
    return dict(status='recorded_preclose_identity_lift_follow_verified' if verified else 'verification_inconclusive',
        grasp_success_verified=verified,source_trial=str(directory),original_trial_status_preserved=trial['status'],
        accepted_after_frames=len(rows),after_span_s=span,after_center_spread_m=spread,
        object_palm_relative_spread_m=relative_spread,
        actual_lift_completed_at_ns=completed,verification_window_end_ns=window_end,
        excluded_transition_captures=transition,
        before_capture=str(before_path),opening_rad=opening,rows=rows,rejected=rejected,
        recording=str(recording),bound_task_id=expected_task_id,bound_epoch=expected_epoch,
        detector_confidence_invented=False,identity_rediscovered_by_color=False,
        motion_authorized=False,robot_commands_sent=False,protocol_success_sent=False,
        limitation='Recorded verification of this pre-identified apple/contact/lift only. NOT open-world color recognition, a live lease, broad success rate or uninterrupted trial05 completion. Query uses measured palm movement but every accepted center is fitted from current RGBD. Original detector-only failure is preserved.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--recorded-geometry-audit',action='store_true');a=p.parse_args()
    if not a.recorded_geometry_audit or a.output.exists():raise ValueError('explicit_recorded_audit_and_new_output_required')
    result=audit(a.trial)
    with a.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','rejected')}),flush=True)
    if not result['grasp_success_verified']:raise SystemExit(1)


if __name__=='__main__':main()
