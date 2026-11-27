"""New reviewed continuation with existing1280 observer option; not old-run replay.

All live class/depth/closure/lift/release gates remain. No threshold/model change.
Full960 trial remains failed; this is explicitly a reviewed continuation test.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
from carry_place_reviewed_grasp_20261018 import (
    code,held_audit,palm_rotation,read,LiveBackend,map_pose,RobotLock,check_inputs,write_json)
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args()
    if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('explicit_simulated_reviewed_continuation_required')
    base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');source=base/'automatic-apple-r11b-20261018-01'
    failed=read(source/'result.json')
    if failed.get('status')!='stopped' or failed.get('stage')!='grasp_and_lift':raise ValueError('unexpected_stop')
    if (source/'grasp/close-lift').exists():raise ValueError('prior_closure_may_have_occurred_no_repeat')
    stop=read(base/'apple-r11b-full-stop-state-20261018-01/000005/snapshot.json');old_pose=map_pose(stop)
    config=read(code/'config/apple_transfer_auto_v1.json')
    if config['policy']['max_placement_step_m'] is not None or config['policy']['edge_margin_m']!=.03 or config['policy']['edge_uncertainty_m']!=.01:
        raise ValueError('requested_r11b_policy_not_installed')
    out=base/'apple-r11b-reviewed1280-transfer-20261018-01';task='reviewed-r11b-1280-'+uuid.uuid4().hex
    with RobotLock():
        inputs=check_inputs(config);prior=read(source/'input-files-audit.json')
        if any(inputs[k]!=prior[k] for k in ('environment_sha256','semantic_map_sha256','model_sha256')):
            raise ValueError('assets_changed_since_source_identification')
        out.mkdir(exist_ok=False)
        report=dict(status='running',run_kind='NEW_reviewed_1280_grasp_carry_r11b_placement',task_id=task,
            stopped_full_trial_preserved=str(source),confirmed_stages=[],automatic_retry=False,automatic_resume=False,
            source_approaches_replayed=False,full_uninterrupted_transfer_verified=False,
            live_source_imgsz=1280,classification_threshold_unchanged=True,
            default_full_entry_still_960=True,task_finished_sent=False,does_not_exist_sent=False)
        with (out/'journal.jsonl').open('x') as log:
            def record(value):
                log.write(json.dumps(dict(time_ns=time.time_ns(),**value),allow_nan=False)+'\n');log.flush();os.fsync(log.fileno())
            b=LiveBackend(config,inputs);b.bind(out,task,record)
            write_json(out/'config.snapshot.json',config);b.evidence('input-files-audit',inputs)
            def step(name,operation):
                b.stage=name;report['stage']=name;write_json(out/'status.json',report)
                record(dict(event='intent',stage=name));print(json.dumps(dict(stage=name,status='starting')),flush=True)
                value=operation()
                if value.get('verified') is not True:raise ValueError('stage_not_verified:'+name)
                report['confirmed_stages'].append(name);record(dict(event='confirmed',stage=name,result=value))
                write_json(out/'status.json',report)
            def precheck():
                b.initial_capture=b.capture('open-source-precheck');s=read(b.initial_capture/'snapshot.json');pose=map_pose(s)
                expected=dict(arm_lift_joint=stop['joints']['arm_lift_joint'],arm_flex_joint=-1.7,arm_roll_joint=0.,
                    wrist_flex_joint=.1292036732051034,wrist_roll_joint=0.,hand_motor_joint=.65,head_pan_joint=0.,head_tilt_joint=-.5)
                if any(abs(s['joints'][k]-v)>.006 for k,v in expected.items()):raise ValueError('prepared_open_pose_changed')
                if math.dist(pose[:2],old_pose[:2])>.02 or abs(pose[2]-old_pose[2])>.02 or b.room_at(pose)!='bedroom':
                    raise ValueError('reviewed_source_base_changed')
                b.grasp_plan=dict(contact_start_lift_m=s['joints']['arm_lift_joint'])
                return b.evidence('current-open-pose',dict(capture=str(b.initial_capture),pose=pose,
                    joints=s['joints'],reference_is_not_live_identity_lease=True))
            def close_lift():
                b.recording=out/'source-observe';b.epoch=task+'-source-observe';b.raw_observer=False
                command=[sys.executable,str(code/'capture_grasp_scene.py'),str(b.recording),'--hand-geometry','--map-geometry',
                    '--tracking-epoch',b.epoch,'--duration','300','--interval','.1','--compact-progress',
                    '--inference-python',config['model_python'],'--registry',config['registry'],'--inference-imgsz','1280',
                    '--environment-yaml',config['environment'],'--expected-layout','LayoutA','--target-room','bedroom',
                    '--identity-tracking-task',task,'--confirm-apple-roi']
                b.observer_log=(out/'source-observe.log').open('x')
                b.observer=subprocess.Popen(command,stdout=b.observer_log,stderr=subprocess.STDOUT,start_new_session=True)
                record(dict(event='observation_start',stage=b.stage,command=command,robot_commands_sent=False))
                path=out/'grasp/close-lift'
                b.command([code/'run_apple_close_lift_trial.py','--run','--model-python',config['model_python'],
                    '--registry',config['registry'],'--environment',config['environment'],
                    '--map-pose',*map_pose(read(b.initial_capture/'snapshot.json')),'--room','bedroom',
                    '--contact-start-lift',b.grasp_plan['contact_start_lift_m'],
                    '--observer-recording',b.recording,'--observer-epoch',b.epoch,'--tracking-task-id',task,
                    '--verify-preclose-rgbd-follow','--motion-profile',config['motion_profile'],'--output',path],
                    path/'result.json','effective_grasp_success_verified',True)
                return dict(verified=True,evidence=str(path/'result.json'))
            def held(name):
                capture=b.capture(name);snapshot=read(Path(b.held['verified_capture'])/'snapshot.json')
                offset=palm_rotation(snapshot).T@np.asarray(b.held['center_minus_palm_m'])
                value=held_audit(b,capture,offset,b.held['radius_m'],[0.,0.,0.,-math.pi/2,0.])
                return b.evidence(name+'-audit',value)
            def retreat():
                capture=b.capture('before-source-retreat')
                path=b.adjust('source-clear-retreat',map_pose(read(capture/'snapshot.json')),[-.25,0.,0.],
                    [0.,0.,0.,-math.pi/2,0.],b.holding)
                return dict(verified=True,evidence=path)
            try:
                step('precheck_current_open_pose',precheck)
                step('new_close_lift_1280',close_lift)
                step('verify_grasp',b.verify_grasp)
                step('retract_holding',b.retract_holding)
                step('verify_retracted_holding',lambda:held('held-after-retract'))
                step('source_clear_retreat',retreat)
                step('navigate_destination',b.navigate_destination)
                step('verify_destination_holding',lambda:held('held-at-destination'))
                for name in ('observe_surface','plan_placement','prepare_placement','approach_placement',
                             'check_release','release','withdraw','verify_placement'):
                    step(name,getattr(b,name))
                report.update(status='reviewed_r11b_grasp_carry_placement_verified',placement_verified=True,
                    new_policy_live_placement_verified=True)
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped',reason=str(exc) or 'interrupted',placement_verified=False,
                    requires_new_physical_state_audit=True)
                record(dict(event='stopped',stage=report.get('stage'),reason=report['reason']))
            finally:
                b.close();write_json(out/'result.json',report);write_json(out/'status.json',report)
            print(json.dumps(report),flush=True)
            if report['status']!='reviewed_r11b_grasp_carry_placement_verified':raise SystemExit(1)


if __name__=='__main__':main()
