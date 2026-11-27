"""Read-only r9 regression on a stopped recorded source approach, never a live lease."""
import argparse
import json
import math
from pathlib import Path

import yaml

from apple_approach_lease import validate_closure
from apple_roi_confirmation import identity_evidence
from apple_tracking_observer import TrackingObserver
from apple_transfer_auto_plan import file_hash, plan_grasp
from apple_worker_client import Worker
from hand_camera_ray import rotation


def read(path):
    return json.loads(Path(path).read_text())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recorded-only',action='store_true')
    parser.add_argument('--source-run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not args.recorded_only or args.output.exists():
        raise ValueError('recorded_only_and_new_output_required')
    root=args.source_run.resolve()
    trial=read(root/'result.json')
    if trial.get('status')!='stopped' or trial.get('stage')!='grasp_and_lift':
        raise ValueError('stopped_source_approach_required')
    config=read(root/'config.snapshot.json')
    environment=yaml.safe_load(Path(config['environment']).read_text())
    inputs=read(root/'input-files-audit.json')
    if file_hash(config['environment'])!=inputs['environment_sha256']:
        raise ValueError('recorded_environment_binding_changed')
    if file_hash(config['semantic_map'])!=inputs['semantic_map_sha256']:
        raise ValueError('recorded_semantic_map_binding_changed')
    semantic=read(config['semantic_map'])
    table=next(d for d in semantic['destinations'] if d['id']==config['source_table_id'])
    original_plan=read(root/'measured-grasp-plan.json')
    initial=Path(original_plan['capture']).resolve()
    recording=(root/'source-observe').resolve()
    if initial.parent!=recording:
        raise ValueError('source_plan_capture_outside_original_recording')
    rebuilt=plan_grasp(read(initial/'observation.json'),read(initial/'snapshot.json'),table,
        dict(target_base_m=original_plan['measured_target_base_m'],stamp_ns=original_plan['source_depth_stamp_ns']),config)
    if abs(rebuilt['palm_height_m']-original_plan['palm_height_m'])>1e-9:
        raise ValueError('original_source_height_plan_changed')
    selected=sorted(recording.glob('*/observation.json'))[-5:]
    if len(selected)!=5:raise ValueError('five_original_near_grasp_frames_required')
    bound=[root/'result.json',root/'measured-grasp-plan.json']
    bound += [p for observation in selected for p in (
        observation, observation.parent/'snapshot.json', observation.parent/'rgb.png', observation.parent/'depth_m.npy')]
    hashes={str(path):file_hash(path) for path in bound}
    args.output.mkdir(parents=True)
    rows=[]
    first=read(selected[0]);epoch=first['tracking_epoch'];task=trial['task_id']
    observer=TrackingObserver(task,epoch,environment,'bedroom',near_grasp_roi=True)
    worker=Worker(config['model_python'],config['registry'],args.output/'worker.log',imgsz=960,confirm_apple_roi=True)
    try:
        if worker.ready['model_sha256']!=inputs['model_sha256']:
            raise ValueError('recorded_model_binding_changed')
        for path in selected:
            original=read(path);snapshot=read(path.parent/'snapshot.json')
            result=worker.infer(path.parent)
            # Original timestamps and the original result-age clock are used
            # only for an explicit historical replay. Nothing is retimestamped
            # to now, saved as a live observation, or provided to a controller.
            now=min(snapshot['streams'][k]['stamp_ns'] for k in ('rgb','depth'))
            now+=round(original['timing']['capture_to_result_age_s']*1e9)
            result.update(timing=original['timing'],tracking_epoch=epoch,identity_tracking_task_id=task)
            result['identity_tracking']=observer.observe(result,snapshot,now)
            roi=identity_evidence(result,snapshot)
            tf=snapshot['tracking_transforms']['odom_from_base_at_depth'];r=rotation(tf['quaternion'])
            closure=None;closure_error=None
            try:
                closure=validate_closure(result,snapshot,now_ns=now,epoch=epoch,task_id=task,
                    current_odom=[*tf['translation'][:2],math.atan2(r[1,0],r[0,0])],
                    palm_height=rebuilt['palm_height_m'],expected_room='bedroom')
            except ValueError as exc:
                closure_error=str(exc)
            proposals=result.get('tracking_proposals',[])
            rows.append(dict(capture=str(path.parent),original_identity=original['identity_tracking'],
                original_support=[p.get('support') for p in original['tracking_proposals']],
                recomputed_support=[p.get('support') for p in proposals],
                original_full_frame_confidences=[p['detection']['confidence'] for p in original['tracking_proposals']],
                recomputed_full_frame_confidences=[p['detection']['confidence'] for p in proposals],
                roi=roi,recomputed_identity=result['identity_tracking'],
                closure_check=closure,closure_error=closure_error,
                clock='Original capture + original result age; historical only, NOT live'))
    finally:
        worker.close()
    if hashes!={str(path):file_hash(path) for path in bound}:
        raise ValueError('original_recording_modified')
    passed=len(rows)>=3 and all(row['closure_check'] is not None for row in rows[-3:])
    report=dict(status='recorded_gate_pruning_replay_passed' if passed else 'recorded_replay_inconclusive',
        original_trial_status_preserved=trial['status'],source_trial=str(root),
        initial_source_support_and_height_still_verified=True,original_inputs_unchanged=True,
        final_three_recorded_closure_checks_passed=passed,rows=rows,
        robot_commands_sent=False,live_lease_created=False,full_automatic_transfer_verified=False,
        limitation='Saved-frame model/geometry/identity regression only. Not live feedback performance, contact, lift or placement acceptance.')
    with (args.output/'result.json').open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
    for row in rows:
        print(json.dumps(dict(capture=row['capture'],roi_confidence=row['roi']['confidence'] if row['roi'] else None,
            identity=row['recomputed_identity']['status'],closure_check_passed=row['closure_check'] is not None,
            closure_error=row['closure_error'])),flush=True)
    if not passed:raise SystemExit(1)


if __name__=='__main__':main()
