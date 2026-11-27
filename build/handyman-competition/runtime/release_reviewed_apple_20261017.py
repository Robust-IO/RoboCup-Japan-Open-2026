"""Explicit one-time release/withdraw verification from the reviewed r7 pose.

No approach, source grasp, navigation or interrupted-journal replay is available.
All actual controls use the existing tested backend, including publish-time gate.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

ROOT=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
OUTPUT=ROOT/'apple-reviewed-release-20261017-01'
CHECKPOINT=ROOT/'apple-single-approach-progress-20261017.json'
SOURCE_PLAN=ROOT/'apple-destination-plan-20261017-01/placement-plan.json'


def validate_recorded_context(checkpoint,action,audit,grasp,plan):
    if (checkpoint.get('placement_approach_already_executed_do_not_repeat') is not True
            or checkpoint.get('hand_or_release_command_sent') is not False
            or checkpoint.get('current_robot',{}).get('holding_object') is not True):
        raise ValueError('reviewed_held_pre_release_context_required')
    if (action.get('status')!='endpoint_verified' or action.get('command_sent') is not True
            or action.get('single_placement_approach') is not True
            or action.get('grip_commands_sent') is not False):
        raise ValueError('completed_single_approach_required')
    if (audit.get('holding_object_verified') is not True or audit.get('samples',0)<3
            or grasp.get('grasp_success_verified') is not True):
        raise ValueError('independently_verified_held_identity_required')
    if (plan.get('destination_id')!='square_low_table#living_room'
            or abs(plan['holding_opening_rad']-checkpoint['current_robot']['hand_motor'])>1e-8):
        raise ValueError('reviewed_destination_or_opening_mismatch')
    last=audit['rows'][-1]
    if (last['stamp_ns']!=checkpoint['current_robot']['source_depth_stamp_ns']
            or math.dist(last['base_map_pose'][:2],checkpoint['current_robot']['base_map_pose'][:2])>.005):
        raise ValueError('reviewed_capture_mismatch')
    return True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true')
    args=parser.parse_args()
    if not args.run:
        print(json.dumps(dict(status='plan_only',stages=['new_owned_release_preflight','check_release','release','withdraw','verify_placement'],robot_commands_sent=False)))
        return
    if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('explicit_local_simulation_environment_required')
    if OUTPUT.exists():
        raise ValueError('reviewed_release_output_already_exists_no_replay')
    sys.path.insert(0,str(CODE))
    from apple_transfer_auto_backend import LiveBackend,read
    from apple_transfer_automation import RobotLock,check_inputs,write_json
    from apple_transfer_auto_plan import map_pose,fresh_snapshot
    from table_release_gate import validate
    def interrupted(_sig,_frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    checkpoint=read(CHECKPOINT)
    action=read(ROOT/'apple-single-forward-action-20261017-01.json')
    audit=read(ROOT/'apple-single-forward-after-20261017-01/prepared-held-audit.json')
    grasp=read(ROOT/'apple-roi-close-lift-20261017-01/independent-rgbd-follow-audit.json')
    plan=read(SOURCE_PLAN)
    validate_recorded_context(checkpoint,action,audit,grasp,plan)
    config=read(CODE/'config/apple_transfer_auto_v1.json')
    task_id='apple-release-reviewed-20261017-01'
    report=dict(status='running',run_kind='explicit_reviewed_release_only',source_checkpoint=str(CHECKPOINT),
                confirmed_stages=[],approach_repeated=False,automatic_retry=False,automatic_resume=False,
                full_transfer_verified=False,task_finished_sent=False,does_not_exist_sent=False)
    with RobotLock():
        inputs=check_inputs(config)
        OUTPUT.mkdir()
        backend=LiveBackend(config,inputs)
        with (OUTPUT/'journal.jsonl').open('x') as journal:
            def record(event):
                event=dict(time_ns=time.time_ns(),**event)
                journal.write(json.dumps(event,allow_nan=False)+'\n');journal.flush();os.fsync(journal.fileno())
            backend.bind(OUTPUT,task_id,record)
            backend.stage='new_owned_release_preflight'
            try:
                write_json(OUTPUT/'config.snapshot.json',config)
                backend.evidence('input-files-audit',inputs)
                backend.evidence('reviewed-source-checkpoint',checkpoint)
                backend.place_plan=copy.deepcopy(plan)
                backend.place_plan.update(task_id=task_id,source_plan=str(SOURCE_PLAN),reviewed_release_only=True)
                backend.place_plan_path=OUTPUT/'placement-plan.json'
                backend.evidence('placement-plan',backend.place_plan)
                backend.evidence('destination-surface',read(ROOT/'apple-destination-plan-20261017-01/destination-surface.json'))
                backend.holding=plan['holding_opening_rad']
                backend.start_observer('release-observe',False)
                rows=[];last=0;deadline=time.monotonic()+20
                while time.monotonic()<deadline:
                    capture,snapshot,_,_=backend.latest()
                    stamp=snapshot['streams']['depth']['stamp_ns']
                    if stamp<=last:
                        time.sleep(.02);continue
                    pose=map_pose(snapshot);before=checkpoint['current_robot']['base_map_pose']
                    if (stamp<=checkpoint['current_robot']['source_depth_stamp_ns']
                            or math.dist(pose[:2],before[:2])>.015
                            or abs(math.atan2(math.sin(pose[2]-before[2]),math.cos(pose[2]-before[2])))>.02):
                        raise ValueError('new_session_or_base_changed_no_release')
                    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
                    try:
                        value=validate(capture,backend.place_plan,now_ns=time.time_ns(),epoch=backend.epoch,
                                       task_id=task_id,current_map_pose=pose,current_joints=[snapshot['joints'][k] for k in names])
                        fresh_snapshot(snapshot,time.time_ns(),backend.epoch)
                    except ValueError as exc:
                        if str(exc) not in ('fresh_synchronised_rgbd_required','fresh_joint_snapshot_required'):
                            raise
                        time.sleep(.02);continue
                    last=stamp;rows.append(value)
                    if len(rows)>=3 and (rows[-1]['source_stamp_ns']-rows[0]['source_stamp_ns'])/1e9>=.4:
                        break
                if len(rows)<3 or (rows[-1]['source_stamp_ns']-rows[0]['source_stamp_ns'])/1e9<.4:
                    raise ValueError('new_multi_frame_release_geometry_not_verified')
                backend.evidence('new-release-preflight',dict(verified=True,samples=rows,new_epoch=backend.epoch,
                    independent_source_identity=str(ROOT/'apple-roi-close-lift-20261017-01/independent-rgbd-follow-audit.json'),
                    no_approach_or_grasp_repeated=True))
                report['confirmed_stages'].append('new_owned_release_preflight')
                for stage in ('check_release','release','withdraw','verify_placement'):
                    report['stage']=stage
                    write_json(OUTPUT/'status.json',report)
                    record(dict(event='intent',stage=stage))
                    print(json.dumps(dict(stage=stage,status='starting')),flush=True)
                    result=backend.perform(stage)
                    if result.get('verified') is not True:
                        raise ValueError('stage_not_verified:'+stage)
                    record(dict(event='confirmed',stage=stage,result=result))
                    report['confirmed_stages'].append(stage)
                report.update(status='reviewed_release_and_placement_verified',placement_verified=True)
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped',reason=str(exc) or 'interrupted',requires_fresh_physical_state_audit=True)
                record(dict(event='stopped',stage=backend.stage,reason=report['reason']))
            finally:
                backend.close()
                write_json(OUTPUT/'result.json',report)
                write_json(OUTPUT/'status.json',report)
                print(json.dumps(report),flush=True)
    if report['status']!='reviewed_release_and_placement_verified':
        raise SystemExit(1)


if __name__=='__main__':main()
