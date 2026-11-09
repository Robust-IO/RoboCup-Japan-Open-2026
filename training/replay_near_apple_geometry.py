"""Explicit historical-clock replay, no ROS imports, publishers or raw-data edits."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image
import yaml

from apple_approach_lease import validate_closure
from apple_tracking_observer import TrackingObserver
from near_apple_geometry import PALM_HEIGHT_OFFSET, recorded_candidate
from hand_camera_ray import rotation


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(recording, environment, output, gap=.15):
    recording=Path(recording);output=Path(output)
    if output.exists():raise ValueError('new_replay_output_required')
    frames=sorted(recording.glob('*/observation.json'))
    if not frames:raise ValueError('recorded_observations_required')
    first=json.loads(frames[0].read_text());task=first['identity_tracking_task_id'];epoch=first['tracking_epoch']
    env=yaml.safe_load(Path(environment).read_text())
    observers=[TrackingObserver(task,epoch,env,'bedroom',near_grasp_roi=True,near_geometry=enabled,near_geometry_gap_m=gap)
               for enabled in (False,True)]
    protected=[p.parent/name for p in frames for name in ('observation.json','snapshot.json','rgb.png','depth_m.npy')]
    failed_result=recording.parent/'result.json'
    if failed_result.exists():protected.append(failed_result)
    original_hashes={str(p):digest(p) for p in protected}
    output.mkdir(parents=True,exist_ok=False);rows=[];final_geometry=None;final_closure=None
    for index,path in enumerate(frames):
        original=json.loads(path.read_text());snapshot=json.loads((path.parent/'snapshot.json').read_text())
        # Replay clock is explicit; image/joint/TF stamps are NEVER rewritten.
        now=max(original['timing'].get('selection_now_ns',0),snapshot['joint_stamp_ns'],
                *(snapshot['streams'][k]['stamp_ns'] for k in ('rgb','depth')))+10_000_000
        old=observers[0].observe(copy.deepcopy(original),snapshot,now,capture=path.parent)
        start=time.perf_counter();result=None
        if observers[1].geometry_ready(snapshot):
            rgb=np.asarray(Image.open(path.parent/'rgb.png').convert('RGB'))
            depth=np.load(path.parent/'depth_m.npy',allow_pickle=False)
            result=observers[1].observe_geometry(rgb,depth,snapshot,now)
        if result is None:
            current=observers[1].observe(copy.deepcopy(original),snapshot,now,capture=path.parent)
        else:
            current=result['identity_tracking']
        elapsed=time.perf_counter()-start
        row=dict(frame=path.parent.name,depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
            replay_now_ns=now,replay_clock_only=True,original_apple_detections=len([d for d in original['detections'] if d['name']=='apple']),
            old_status=old['status'],old_reason=old.get('reason'),new_status=current['status'],new_reason=current.get('reason'),
            detector_required_for_new_frame=result is None,geometry_processing_including_image_load_s=elapsed)
        if current['status']=='near_geometry_tracked':
            observation=dict(result,tracking_epoch=epoch,timing=dict(freshness_passed=True))
            candidate=recorded_candidate(observation,snapshot,rgb,depth,directory=recording)
            row.update(measured_center_base_m=candidate['center_m'],radius_m=candidate['radius_m'],
                residual_p95_m=candidate['fit_residual_p95_m'],pixels=candidate['points'],detector_confidence=None)
            final_geometry=dict(capture=str(path.parent),observation=observation,candidate=candidate)
            tf=snapshot['tracking_transforms']['odom_from_base_at_depth'];r=rotation(tf['quaternion'])
            try:
                evidence=validate_closure(observation,snapshot,now_ns=now,epoch=epoch,task_id=task,
                    current_odom=[*tf['translation'][:2],float(np.arctan2(r[1,0],r[0,0]))],
                    palm_height=snapshot['joints']['arm_lift_joint']+PALM_HEIGHT_OFFSET,expected_room='bedroom')
                row['recorded_closure_geometry_valid']=True;final_closure=dict(capture=str(path.parent),evidence=evidence)
            except ValueError as exc:row.update(recorded_closure_geometry_valid=False,closure_reason=str(exc))
        rows.append(row)
        if index%20==0:print(json.dumps(row),flush=True)
    unchanged=all(digest(Path(path))==value for path,value in original_hashes.items())
    rescued=[r for r in rows if r['original_apple_detections']==0 and r['new_status']=='near_geometry_tracked']
    geometry_rows=[r for r in rows if r['new_status']=='near_geometry_tracked']
    okay=(unchanged and bool(rescued) and bool(final_closure)
          and rows[-1]['new_status']=='near_geometry_tracked' and rows[-1].get('recorded_closure_geometry_valid') is True)
    report=dict(status='recorded_near_geometry_replay_passed' if okay else 'recorded_replay_inconclusive',
        recording=str(recording),frames=len(rows),geometry_frames=len(geometry_rows),
        no_apple_detector_frames_rescued=len(rescued),first_handoff_frame=geometry_rows[0]['frame'] if geometry_rows else None,
        last_frame=rows[-1],last_geometry_evidence=final_geometry,last_closure_geometry=final_closure,
        rows=rows,original_recordings_and_failed_result_hashes_unchanged=unchanged,
        historical_clock_only=True,stamps_rewritten=False,robot_commands_sent=False,
        detector_or_model_changed=False,live_motion_validated=False,
        warning='Historical replay can validate pixel/depth association and closure geometry only. '
                'Not a live lease, live timing proof, action authorization or full-run success.')
    with (output/'result.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    with (output/'original-hashes.json').open('x') as f:json.dump(original_hashes,f,indent=2)
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','last_geometry_evidence','last_closure_geometry')}),flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--recorded-only',action='store_true')
    p.add_argument('--recording',type=Path,required=True);p.add_argument('--environment',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not a.recorded_only:raise ValueError('explicit_recorded_only_required')
    result=replay(a.recording,a.environment,a.output)
    raise SystemExit(0 if result['status']=='recorded_near_geometry_replay_passed' else 1)
