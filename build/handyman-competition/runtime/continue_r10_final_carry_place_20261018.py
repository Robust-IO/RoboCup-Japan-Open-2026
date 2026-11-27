"""Explicit new live test of r10 from actual held corridor-stop state; no replay."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
from carry_place_reviewed_grasp_20261018 import (
    code,held_audit,palm_rotation,palm_in_base,read,LiveBackend,map_pose,
    RobotLock,check_inputs,write_json)
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args()
    if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('explicit_simulation_run_required')
    base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
    source=base/'apple-gate-pruning-live-carry-place-20261018-01'
    stop=read(base/'apple-final-stop-audit-20261018-01/post-stop-audit.json')
    deployed=read(base/'apple-navigation-feedback-r10-20261018-deployment.json')
    if stop['holding_audit'].get('held_object_verified') is not True or deployed.get('test_exit')!=0:
        raise ValueError('verified_stop_or_tested_patch_missing')
    from navigation_feedback_gate import REACQUIRE_BUDGET_S,MAX_SAMPLE_AGE_S
    if REACQUIRE_BUDGET_S!=.15 or MAX_SAMPLE_AGE_S!=.7:raise ValueError('wrong_policy')
    config=read(code/'config/apple_transfer_auto_v1.json')
    out=base/'apple-navigation-feedback-r10-live-place-20261018-01'
    task='reviewed-r10-final-place-'+uuid.uuid4().hex
    with RobotLock():
        inputs=check_inputs(config);prior=read(source/'input-files-audit.json')
        if any(inputs[k]!=prior[k] for k in ('environment_sha256','semantic_map_sha256','model_sha256')):
            raise ValueError('assets_changed_since_actual_grasp')
        proof=read(source/'source-grasp-recomputed.json');last=proof['rows'][-1]
        s=read(Path(last['capture'])/'snapshot.json');center=np.asarray(last['center_measurement']['center_m'])
        offset=palm_rotation(s).T@(center-palm_in_base(s));radius=last['center_measurement']['radius_m']
        out.mkdir(exist_ok=False)
        report=dict(status='running',run_kind='NEW_reviewed_r10_final_navigation_and_placement',task_id=task,
            confirmed_stages=[],automatic_retry=False,automatic_resume=False,
            previous_stopped_runs_preserved=True,source_actions_or_completed_waypoints_replayed=False,
            full_uninterrupted_transfer_verified=False,task_finished_sent=False,does_not_exist_sent=False)
        with (out/'journal.jsonl').open('x') as log:
            def record(value):
                log.write(json.dumps(dict(time_ns=time.time_ns(),**value),allow_nan=False)+'\n');log.flush();os.fsync(log.fileno())
            b=LiveBackend(config,inputs);b.bind(out,task,record)
            b.held=read(source/'verified-held-geometry.json');b.holding=b.held['holding_opening_rad']
            write_json(out/'config.snapshot.json',config);b.evidence('input-files-audit',inputs)
            def step(name,operation):
                b.stage=name;report['stage']=name;write_json(out/'status.json',report)
                record(dict(event='intent',stage=name));print(json.dumps(dict(stage=name,status='starting')),flush=True)
                value=operation()
                if value.get('verified') is not True:raise ValueError('stage_not_verified:'+name)
                report['confirmed_stages'].append(name);record(dict(event='confirmed',stage=name,result=value))
                write_json(out/'status.json',report)
            def audit_current(name,at_stop=False):
                capture=b.capture(name);snap=read(capture/'snapshot.json')
                value=held_audit(b,capture,offset,radius,[0.,0.,0.,-math.pi/2,0.])
                pose=map_pose(snap)
                if b.room_at(pose)!='living_room':raise ValueError('actual_room_not_living_room')
                if at_stop and (math.dist(pose[:2],stop['last_sample']['map_pose'][:2])>.08
                    or abs(math.atan2(math.sin(pose[2]-stop['last_sample']['map_pose'][2]),
                                     math.cos(pose[2]-stop['last_sample']['map_pose'][2])))>.05):
                    raise ValueError('robot_changed_since_reviewed_stop')
                return b.evidence(name+'-audit',value)
            def final_navigation():
                path=out/'final-navigation.json'
                value=b.command([code/'navigate_carry_trial.py','--run','--holding-opening',b.holding,
                    '--verify-current-precision','--motion-profile',config.get('motion_profile','baseline'),
                    '--goal',*config['destination_staging_goal'],'--output',path],path,'status','arrived')
                if (value.get('nav_reset') is not True or value.get('goal_precision_changed') is not False
                    or value.get('speed_parameters_restored') is not True or value.get('grip_commands_sent') is not False):
                    raise ValueError('navigation_cleanup_not_verified')
                return dict(verified=True,evidence=str(path))
            try:
                step('verify_current_holding',lambda:audit_current('held-before-final-navigation',True))
                step('navigate_final_goal',final_navigation)
                step('verify_destination_holding',lambda:audit_current('held-at-destination'))
                for name in ('observe_surface','plan_placement','prepare_placement','approach_placement',
                             'check_release','release','withdraw','verify_placement'):
                    step(name,getattr(b,name))
                report.update(status='reviewed_r10_final_carry_placement_verified',placement_verified=True)
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped',reason=str(exc) or 'interrupted',placement_verified=False,
                    requires_new_physical_state_audit=True)
                record(dict(event='stopped',stage=report.get('stage'),reason=report['reason']))
            finally:
                b.close();write_json(out/'result.json',report);write_json(out/'status.json',report)
            print(json.dumps(report),flush=True)
            if report['status']!='reviewed_r10_final_carry_placement_verified':raise SystemExit(1)


if __name__=='__main__':main()
