"""Single fresh-run can transfer using the separately verified action adapters.

Code integration does not establish a continuous physical success until a NEW
live run reaches final placement. No historical checkpoint is read in this mode.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import re
import signal
import time
import uuid

from apple_transfer_automation import RobotLock,check_inputs,write_json
from apple_transfer_auto_plan import validate_config
from manipulation_profiles import object_profile

STAGES=('preflight','navigate_source','head_view','grasp_lift','carry_destination','place_verify')
BASELINE=Path(__file__).parent/'config/apple_transfer_auto_v1.json'


def configuration(baseline=None):
    baseline=copy.deepcopy(baseline if baseline is not None else json.loads(BASELINE.read_text()))
    validate_config(baseline)
    return dict(version=1,profile='layouta_can_white_side_to_square_table_v1',
        task=dict(object='canned_juice',source_room='bedroom',destination='square_low_table'),
        object_profile=object_profile('canned_juice'),baseline=baseline,
        scope='One fixed LayoutA can-transfer trial; no Moderator or arbitrary-object execution.')


def run(config,task_id,output,backend,*,owner_fd=None):
    if config!=configuration(config['baseline']):raise ValueError('unsupported_can_transfer_configuration')
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,100}',task_id):raise ValueError('invalid_task_id')
    if backend.mode not in ('live','offline_test'):raise ValueError('unsupported_can_backend')
    root=Path(output);root.mkdir(parents=True,exist_ok=False)
    meta=dict(version=1,run_id=task_id,object='canned_juice',run_root=str(root.resolve()),
        started_ns=time.time_ns(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),mode=backend.mode)
    write_json(root/'run-meta.json',meta);write_json(root/'config.snapshot.json',config)
    report=dict(status='running',mode=backend.mode,task_id=task_id,class_name='canned_juice',confirmed_stages=[],
        physical_transfer_verified=False,uninterrupted_transfer_verified=False,task_objective_completed=False,task_finished_sent=False,
        does_not_exist_sent=False,automatic_action_retry=False,automatic_resume=False,
        historical_checkpoint_used=False,navigation_tolerances_changed=False,robot_commands_sent=False)
    with (root/'journal.jsonl').open('x') as stream:
        def record(event):
            value=dict(time_ns=time.time_ns(),**event);stream.write(json.dumps(value,allow_nan=False)+'\n');stream.flush();os.fsync(stream.fileno())
            print(json.dumps(value),flush=True)
        try:
            backend.bind_run(root,task_id,config,record,owner_fd)
            for stage in STAGES:
                report['stage']=stage;write_json(root/'result.json',report);record(dict(event='intent',stage=stage))
                response=backend.perform(stage)
                if not isinstance(response,dict) or response.get('verified') is not True:raise ValueError('can_stage_not_verified:'+stage)
                path=Path(response['evidence']).resolve()
                if not path.is_relative_to(root.resolve()) or not path.is_file():raise ValueError('evidence_outside_current_run')
                evidence=json.loads(path.read_text())
                if backend.mode=='live' and evidence.get('synthetic') is True:raise ValueError('synthetic_evidence_in_live_run')
                report['confirmed_stages'].append(stage);record(dict(event='confirmed',stage=stage,evidence=str(path)))
                if stage=='place_verify':report['robot_ready_for_next_task']=bool(response.get('robot_ready_for_next_task',True))
            physical=backend.mode=='live';ready=report['robot_ready_for_next_task']
            report.update(status=('automatic_can_transfer_verified' if ready else 'automatic_can_transfer_verified_cleanup_required')
                if physical else 'offline_can_flow_verified_not_physical_transfer',
                physical_transfer_verified=physical,uninterrupted_transfer_verified=physical,
                task_objective_completed=physical,robot_commands_sent=physical)
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped_requires_actual_state_review' if backend.mode=='live' else 'stopped_offline',
                reason=str(exc) or 'interrupted',requires_fresh_state_audit=True)
            record(dict(event='stopped',stage=report.get('stage'),reason=report['reason'],automatic_retry=False))
        finally:
            try:backend.close()
            except Exception as exc:
                report.update(cleanup_error=str(exc),robot_ready_for_next_task=False,requires_fresh_state_audit=True)
                if report['physical_transfer_verified']:report['status']='automatic_can_transfer_verified_cleanup_required'
            report['completed_ns']=time.time_ns();report['elapsed_s']=(report['completed_ns']-meta['started_ns'])/1e9
            if backend.mode=='live':report['robot_commands_sent']=bool(getattr(backend,'action_intents',0))
            write_json(root/'result.json',report)
    return report


class OfflineBackend:
    mode='offline_test'
    def __init__(self,fail=None):self.fail=fail;self.calls=[]
    def bind_run(self,root,task_id,config,record,owner_fd):self.root=root;self.record=record
    def perform(self,stage):
        self.calls.append(stage)
        if stage==self.fail:raise RuntimeError('injected_failure:'+stage)
        path=self.root/(stage+'.json');write_json(path,dict(synthetic=True,physical_action_executed=False,stage=stage))
        return dict(verified=True,evidence=str(path),robot_ready_for_next_task=True)
    def close(self):pass


def live_backend(config):
    # ROS and motion-capable imports are delayed until explicit --run.
    import subprocess
    import sys
    from apple_transfer_auto_backend import LiveBackend,stop_group,read
    from can_transfer_context import build_context
    class Backend(LiveBackend):
        mode='live'
        def bind_run(self,root,task_id,config,record,owner_fd):
            if owner_fd is None:raise ValueError('parent_controller_owner_required')
            self.bind(root,task_id,record);self.owner_fd=owner_fd;self.action_intents=0
        def command(self,arguments,result_path,expected_key=None,expected_value=None,timeout=None):
            self.serial+=1;command=[sys.executable,*map(str,arguments)]
            if '--run' in command:self.action_intents+=1
            self.record(dict(event='command_intent',stage=self.stage,sequence=self.serial,command=command))
            with (self.output/f'command-{self.serial:03d}.log').open('x') as log:
                self.active_child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,pass_fds=(self.owner_fd,))
                try:code=self.active_child.wait(timeout=timeout)
                except BaseException:
                    stop_group(self.active_child);self.active_child=None;raise
            self.active_child=None;result=read(result_path)
            if code or expected_key is not None and result.get(expected_key)!=expected_value:
                raise RuntimeError('can_child_failed:'+str(result_path)+':'+str(result.get('reason',result.get('error',result.get('status',code)))))
            self.record(dict(event='command_confirmed',stage=self.stage,sequence=self.serial,evidence=str(result_path)))
            return result
        def preflight(self):
            audit=check_inputs(self.config);self.evidence('input-files-audit',audit)
            path=self.output/'preflight.json'
            self.ready=self.command([self.code/'inspect_transfer_ready.py','--output',path,'--efficient-transfer'],path,'ready',True,25)
            self.start_room=self.room_at(self.ready['base_map_pose'])
            if self.start_room not in ('living_room','bedroom'):raise ValueError('start_outside_reviewed_source_route')
            self.initial_opening=self.ready['joints']['hand_motor_joint']
            return dict(verified=True,evidence=str(path))
        def adapter(self,name,script,flags,status):
            path=self.output/name
            arguments=[self.code/script,'--run','--output',path,'--owner-lock-fd',self.owner_fd,'--owner-run-root',self.output,*flags]
            result=self.command(arguments,path/'result.json','status',status)
            return result,dict(verified=True,evidence=str(path/'result.json'))
        def grasp_lift(self):
            result,response=self.adapter('grasp','run_canned_grasp_trial.py',['--simulation-ignore-collision'],'can_grasp_and_lift_verified')
            self.context=build_context(self.output);return response
        def carry_destination(self):
            result,response=self.adapter('carry','run_canned_carry_trial.py',['--navigate-destination','--context',self.context],
                'can_retraction_and_destination_navigation_verified')
            if not result.get('retraction_verified') or not result.get('destination_navigation_verified'):raise ValueError('actual_can_carry_not_verified')
            return response
        def place_verify(self):
            result,response=self.adapter('place','run_canned_place_trial.py',['--context',self.context],'reviewed_can_placement_verified')
            if not result.get('placement_success_verified') or result.get('release_commands_sent')!=1:raise ValueError('actual_can_placement_not_verified')
            return dict(response,robot_ready_for_next_task=result.get('robot_ready_for_next_task',False))
    return Backend(config['baseline'])


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group();mode.add_argument('--run',action='store_true');mode.add_argument('--offline-smoke-test',action='store_true');mode.add_argument('--check-inputs',action='store_true')
    p.add_argument('--output',type=Path);p.add_argument('--task-id');p.add_argument('--confirm-layout');p.add_argument('--confirm-localization',action='store_true');p.add_argument('--simulation-ignore-collision',action='store_true')
    args=p.parse_args(argv);config=configuration()
    if args.check_inputs:print(json.dumps(check_inputs(config['baseline'])));return
    if not args.run and not args.offline_smoke_test:
        print(json.dumps(dict(status='can_transfer_plan_only',task=config['task'],stages=list(STAGES),robot_commands_sent=False,
            historical_checkpoints_required=False,entry_available=True,continuous_physical_trial_verified=False,
            navigation_parameters_changed=False,maximum_grasp_corrections=5)));return
    if args.output is None:p.error('new output directory required')
    task_id=args.task_id or 'can-auto-'+uuid.uuid4().hex
    if args.offline_smoke_test:result=run(config,task_id,args.output,OfflineBackend())
    else:
        if (args.confirm_layout!='LayoutA' or not args.confirm_localization or not args.simulation_ignore_collision
                or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1'
                or not args.output.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs'))):
            p.error('reviewed simulation LayoutA/localization, domain71/localhost and NEW D-drive output required')
        def interrupted(*_):raise KeyboardInterrupt()
        signal.signal(signal.SIGTERM,interrupted)
        with RobotLock() as lock:result=run(config,task_id,args.output,live_backend(config),owner_fd=lock.stream.fileno())
    print(json.dumps(result))
    if result['status'].startswith('stopped'):raise SystemExit(1)


if __name__=='__main__':main()
