"""Explicit NEW carry/place after this turn's verified grasp; no source action replay."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

import numpy as np
from PIL import Image

code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_auto_backend import LiveBackend,read
from apple_transfer_auto_plan import map_pose
from apple_transfer_automation import RobotLock,check_inputs,write_json
from audit_preclose_identity_lift_follow import audit
from hand_camera_ray import rotation
from tracked_apple_surface import measure
from verify_apple_lift_sequence import palm_in_base


def palm_rotation(s):
    return rotation(s['base_from_hand']['quaternion'])@rotation(s['palm_from_hand']['quaternion']).T


def held_audit(backend,capture,offset_palm,reference_radius,expected_arm):
    rows=[]
    for path in sorted(capture.parent.glob('*/snapshot.json')):
        s=read(path);j=s['joints']
        names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        if any(abs(j[key]-value)>.006 for key,value in zip(names,[*expected_arm,backend.holding])):
            raise ValueError('current_holding_pose_changed')
        expected=palm_in_base(s)+palm_rotation(s)@offset_palm
        value=measure(np.asarray(Image.open(path.parent/'rgb.png').convert('RGB')),
            np.load(path.parent/'depth_m.npy',allow_pickle=False),s,expected)
        if abs(value['radius_m']-reference_radius)>.008:
            raise ValueError('known_held_apple_size_changed')
        actual_offset=palm_rotation(s).T@(np.asarray(value['center_base'])-palm_in_base(s))
        rows.append(dict(capture=str(path.parent),stamp_ns=s['streams']['depth']['stamp_ns'],
            measurement=value,palm_relative_offset_m=actual_offset.tolist(),base_map_pose=map_pose(s)))
    span=(rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9 if len(rows)>1 else 0
    spread=max((float(np.linalg.norm(np.asarray(r['palm_relative_offset_m'])-rows[-1]['palm_relative_offset_m'])) for r in rows),default=1.)
    if len(rows)<3 or span<.4 or spread>.01:
        raise ValueError('current_multiframe_held_geometry_unverified')
    return dict(held_object_verified=True,frames=len(rows),span_s=span,
        palm_relative_center_spread_m=spread,rows=rows,robot_commands_sent=False,
        basis='Verified current grasp identity plus NEW measured RGBD; palm rotation accounted for')


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args()
    if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('explicit_simulated_current_grasp_continuation_required')
    base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
    source=base/'apple-gate-pruning-live-close-lift-20261018-01'
    out=base/'apple-gate-pruning-live-carry-place-20261018-01'
    config=read(code/'config/apple_transfer_auto_v1.json')
    trial=read(source/'result.json')
    if trial.get('effective_grasp_success_verified') is not True or trial.get('observer_owned') is not True:
        raise ValueError('this_turn_actual_verified_owned_grasp_required')
    task='reviewed-carry-place-'+uuid.uuid4().hex
    with RobotLock():
        inputs=check_inputs(config)
        prior=read(base/'apple-gate-pruning-live-close-lift-20261018-01.startup.json')['input_files_audit']
        if any(inputs[k]!=prior[k] for k in ('environment_sha256','semantic_map_sha256','model_sha256')):
            raise ValueError('assets_changed_since_actual_grasp')
        proof=audit(source,expected_epoch=trial['tracking_epoch'],expected_task_id=trial['tracking_task_id'])
        if proof['grasp_success_verified'] is not True:raise ValueError('actual_grasp_proof_recompute_failed')
        row=proof['rows'][-1];s=read(Path(row['capture'])/'snapshot.json')
        candidate=row['center_measurement'];center=np.asarray(candidate['center_m']);palm=palm_in_base(s)
        offset_palm=palm_rotation(s).T@(center-palm)
        before=read(Path(trial['close']['closure_identity_evidence']['evidence']).parent/'snapshot.json')
        grasp_lift=before['joints']['arm_lift_joint'];opening=trial['close']['closure_evidence']['observed_opening_rad']
        out.mkdir(exist_ok=False)
        report=dict(status='running',run_kind='reviewed_new_grasp_carry_place_continuation',task_id=task,
            source_grasp=str(source),confirmed_stages=[],automatic_retry=False,automatic_resume=False,
            old_failed_full_trial_replayed=False,full_uninterrupted_transfer_verified=False,
            task_finished_sent=False,does_not_exist_sent=False)
        with (out/'journal.jsonl').open('x') as log:
            def record(value):
                log.write(json.dumps(dict(time_ns=time.time_ns(),**value),allow_nan=False)+'\n');log.flush();os.fsync(log.fileno())
            b=LiveBackend(config,inputs);b.bind(out,task,record)
            b.grasp_plan=dict(contact_start_lift_m=grasp_lift)
            b.held=dict(center_minus_palm_m=(center-palm).tolist(),
                bottom_below_palm_m=float(palm[2]-center[2]+candidate['radius_m']),
                radius_m=candidate['radius_m'],holding_opening_rad=opening,verified_capture=row['capture'],
                new_identity_verified=False)
            b.holding=opening
            write_json(out/'config.snapshot.json',config);b.evidence('input-files-audit',inputs)
            b.evidence('source-grasp-recomputed',proof);b.evidence('verified-held-geometry',b.held)
            def step(name,operation):
                b.stage=name;report['stage']=name;write_json(out/'status.json',report)
                record(dict(event='intent',stage=name));print(json.dumps(dict(stage=name,status='starting')),flush=True)
                evidence=operation()
                if evidence.get('verified') is not True:raise ValueError('stage_not_verified:'+name)
                report['confirmed_stages'].append(name)
                record(dict(event='confirmed',stage=name,result=evidence))
                write_json(out/'status.json',report)
            def audit_current(name,arm):
                capture=b.capture(name)
                value=held_audit(b,capture,offset_palm,candidate['radius_m'],arm)
                if name=='held-before-retract' and b.room_at(map_pose(read(capture/'snapshot.json')))!='bedroom':
                    raise ValueError('current_source_room_changed')
                return b.evidence(name+'-audit',value)
            def retreat():
                capture=b.capture('before-source-retreat')
                path=b.adjust('source-clear-retreat',map_pose(read(capture/'snapshot.json')),[-.25,0.,0.],
                    [0.,0.,0.,-math.pi/2,0.],opening)
                return dict(verified=True,evidence=path)
            try:
                step('verify_current_holding',lambda:audit_current('held-before-retract',
                    [grasp_lift+.1,-1.7,0.,.1292036732051034,0.]))
                step('retract_holding',b.retract_holding)
                step('verify_retracted_holding',lambda:audit_current('held-after-retract',[0.,0.,0.,-math.pi/2,0.]))
                step('source_clear_retreat',retreat)
                step('navigate_destination',b.navigate_destination)
                step('verify_destination_holding',lambda:audit_current('held-at-destination',[0.,0.,0.,-math.pi/2,0.]))
                for name in ('observe_surface','plan_placement','prepare_placement','approach_placement',
                             'check_release','release','withdraw','verify_placement'):
                    step(name,getattr(b,name))
                report.update(status='reviewed_grasp_carry_placement_verified',placement_verified=True)
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped',reason=str(exc) or 'interrupted',placement_verified=False,
                    requires_new_physical_state_audit=True)
                record(dict(event='stopped',stage=report.get('stage'),reason=report['reason']))
            finally:
                b.close();write_json(out/'status.json',report);write_json(out/'result.json',report)
            print(json.dumps(report),flush=True)
            if report['status']!='reviewed_grasp_carry_placement_verified':raise SystemExit(1)


if __name__=='__main__':main()
