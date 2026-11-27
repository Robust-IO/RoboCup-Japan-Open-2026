"""Unified entry: default observe/plan only; --execute enables bounded grasp.

No per-object grasp type or manually supplied target point is accepted.
Source-table identifies context, target identifies a model detection only.
"""
from competition_paths import is_run_path
import argparse
import json
import os
from pathlib import Path
import signal
import time
import uuid


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--target',required=True)
    p.add_argument('--source-table',required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--hand',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume-plan-from',type=Path,help='explicitly reuse the last selected candidate and its recorded scene')
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true')
    mode.add_argument('--preflight',action='store_true',help='check actual runtime/state only; no model inference or motion')
    p.add_argument('--method',choices=('auto','geometry','templates'),default='auto',help='Prefer verified object method; geometry explicitly selects experimental planner')
    args=p.parse_args()
    from autograsp.methods import select_method,local_command
    args.target=select_method(args.target)['target']
    if args.resume_plan_from and not args.execute:raise ValueError('resume_plan_requires_execute')
    if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('existing_domain71_environment_required')
    if not is_run_path(args.output.resolve()):
        raise ValueError('output_must_be_on_D')
    if args.method=='auto' and not args.preflight:
        method=select_method(args.target)
        if args.resume_plan_from and method['method'] not in ('geometry','geometry_overhead'):
            raise ValueError('historical_generic_plan_not_valid_for_verified_method')
        command=local_command(args.target,args.source_table,args.output,Path(__file__).parent,
                              Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),Path('/__HANDYMAN_PACKAGE__/runtime'))
        if command is not None:
            if not args.execute:
                print(json.dumps(dict(status='verified_method_selected',profile=method,command=command,
                                      robot_motion_requested=False,pose_compatibility='checked_by_current_RGBD_at_execution')))
                return 0
            import subprocess,sys
            print(json.dumps(dict(event='verified_method_dispatch',profile=method,command=command)),flush=True)
            completed=subprocess.run([sys.executable,*command],check=False)
            if completed.returncode:return completed.returncode
            result_path=args.output/'result.json'
            if not result_path.exists():return 2
            outcome=json.loads(result_path.read_text())
            return 0 if outcome.get('grasp_success_verified') is True else 2
    from apple_transfer_automation import RobotLock,write_json
    from apple_worker_client import Worker
    from thin_box_clearance import ThinBoxClearance
    from autograsp.planner import Hand,plan,Policy
    from autograsp.coordinator import Coordinator
    from autograsp.ros_backend import RosBackend,runtime_class
    cfg=json.loads(args.config.read_text())
    rules=json.loads((Path(__file__).parent/'grasp_strategy_policy.json').read_text())
    override=rules.get('target_overrides',{}).get(args.target,{})
    settings={**rules['default'],**override}
    policy=Policy(**{**settings,'allowed_strategies':tuple(settings['allowed_strategies'])})
    table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==args.source_table)
    hand=Hand(json.loads(args.hand.read_text()))
    runtime=worker=None
    def interrupt(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    with RobotLock():
        args.output.mkdir(parents=True,exist_ok=False)
        result=dict(status='starting',execution_enabled=args.execute,target=args.target,
                    source_table=args.source_table,close_commands=0)
        with (args.output/'journal.jsonl').open('x') as journal:
            def record(event):
                journal.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');journal.flush()
                if event.get('event')=='can_closure_sent':result['close_commands']+=1
            import rclpy
            rclpy.init()
            try:
                runtime=runtime_class()(args.output,'autograsp-'+uuid.uuid4().hex,record)
                runtime.sample(profile='state')
                os.environ['HANDYMAN_RESIDENT_MODEL']='1'
                if not args.preflight:
                    worker=Worker(cfg['model_python'],cfg['registry'],args.output/'model.log',imgsz=960)
                backend=RosBackend(runtime,worker,table,hand,ThinBoxClearance().hand,record,policy=policy)
                backend.use_family_templates=args.method=='templates'
                result['initial_state']=runtime.state()
                record(dict(event='explicit_strategy_policy',target=args.target,settings=settings,target_override_applied=bool(override)))
                if args.preflight:
                    result.update(status='runtime_preflight_passed',holding_state=backend.holding_state(),
                                  robot_motion_requested=False)
                elif args.execute:
                    initial=backend.restore_selected_plan(args.resume_plan_from,args.target) if args.resume_plan_from else None
                    result.update(Coordinator(backend,hand,policy=policy,record=record).run(args.target,initial_plan=initial))
                    if result.get('status') == 'grasp_verified':
                        context=backend.held_context
                        context['producer_output']=str(args.output.resolve())
                        write_json(args.output/'held-context.json',context)
                        result['held_context_path']=str(args.output/'held-context.json')
                        result['carry_ready']=False
                else:
                    scene=backend.observe_stopped(args.target)
                    if backend.use_family_templates:
                        from autograsp.templates import select_template
                        policy,selection=select_template(args.target,scene,policy)
                        backend.policy=policy
                        record(dict(event='grasp_template_selected',**selection))
                    decision=plan(scene,hand,policy);write_json(args.output/'plan.json',decision)
                    result.update(status=decision['status'],selected=decision['selected'],robot_motion_requested=False)
            except (Exception,KeyboardInterrupt) as exc:
                result.update(status='stopped_with_error',error_type=type(exc).__name__,reason=str(exc))
                record(dict(event='unhandled_error',reason=str(exc)))
            finally:
                if worker:worker.close()
                if runtime:
                    try:result['last_state']=runtime.state()
                    except Exception:pass
                    runtime.shutdown()
                rclpy.shutdown()
                write_json(args.output/'result.json',result)
                print(json.dumps(result))
        return 0 if result['status'] in ('grasp_verified','runtime_preflight_passed','candidate_selected') else 2


if __name__=='__main__':raise SystemExit(main())
