"""Session1 entry. Default: validate files only. --execute starts actual ROS work."""
import argparse
import json
from pathlib import Path
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--hand',type=Path,default=Path('/__HANDYMAN_PACKAGE__/runtime/autograsp-v1/replay-inputs/hand.json'))
    parser.add_argument('--output',type=Path)
    parser.add_argument('--resume-source-navigation',type=Path)
    parser.add_argument('--resume-room-from',type=Path)
    parser.add_argument('--resume-held-from',type=Path)
    parser.add_argument('--resume-failed-grasp-from',type=Path)
    parser.add_argument('--resume-ready-release-from',type=Path)
    parser.add_argument('--resume-uncertain-from',type=Path)
    parser.add_argument('--resume-released-from',type=Path)
    parser.add_argument('--resume-observation-from',type=Path,help='same running Unity round only; resume before grasp from received-task evidence')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true')
    mode.add_argument('--preflight',action='store_true',help='live read-only sensor/model check; no protocol or motion')
    parser.add_argument('--prepare-initial-empty',action='store_true')
    args=parser.parse_args()
    if args.prepare_initial_empty and (not args.execute or any(v for k,v in vars(args).items() if k.startswith('resume_'))):
        parser.error('initial preparation requires fresh execute without resume')
    config=json.loads(args.config.read_text())
    from live_config import validate
    validate(config)
    if args.execute:
        from live_config import require_container_execution_ready
        require_container_execution_ready(config)
    for key in ('environment','semantic_map','registry','model_python'):
        if not Path(config[key]).exists():raise ValueError('missing_'+key+':'+config[key])
    if not args.hand.is_file():raise ValueError('hand_geometry_missing')
    if not args.execute and not args.preflight:
        print(json.dumps(dict(mode='configuration_check_only',configuration_files_present=True,
                             ros_initialized=False,robot_commands_sent=False,live_validation_passed=False)))
        return 0
    if args.output is None:parser.error('--execute/--preflight requires a new --output directory on D')
    roots=[Path(__file__).parent,Path('/__HANDYMAN_PACKAGE__/runtime/generic-grasp-v1'),
           Path('/__HANDYMAN_PACKAGE__/runtime'),Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')]
    sys.path[:0]=list(map(str,roots))
    import signal
    import uuid
    import rclpy
    from runtime_resources import RuntimeResources
    from assembly import assemble
    owner=RuntimeResources(args.config,args.hand,args.output)
    bundle=None;assembled=None;interrupted=False;protocol_executor=None
    def interrupt(*_):
        nonlocal interrupted
        if assembled is None:raise KeyboardInterrupt()
        interrupted=True
    old={s:signal.signal(s,interrupt) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        bundle=owner.open(prepare_initial_empty=args.prepare_initial_empty)
        if args.preflight:
            from live_preflight import run
            print(json.dumps(run(bundle)))
            return 0
        options={}
        if config.get('session_profile',{}).get('destination',{}).get('mode')=='container_drop':
            from container_live_adapter import ContainerLiveAdapter
            options['container_adapter_factory']=ContainerLiveAdapter
        assembled=assemble(**bundle,session_id='session1-'+uuid.uuid4().hex,**options)
        app=assembled['application'];runtime=bundle['runtime']
        from rclpy.executors import SingleThreadedExecutor
        protocol_executor=SingleThreadedExecutor()
        protocol_executor.add_node(bundle['node'])
        # Current startup feedback, not checkpoint replay. No I_am_ready until
        # localization and empty hand have been observed in this process.
        runtime.sample(profile='state');state=runtime.state()
        (args.output/'startup-state.json').write_text(json.dumps(state,indent=2))
        if bundle['motion_backend'].room_at(state['poses']['map']) is None:
            raise ValueError('initial_room_not_localized')
        from task_outcome_policy import hand_is_open
        if args.resume_room_from:
            if any((args.resume_failed_grasp_from,args.resume_ready_release_from,args.resume_held_from,args.resume_uncertain_from,args.resume_released_from,args.resume_observation_from)):
                raise ValueError('resume_modes_are_exclusive')
            from resume_room_navigation import restore
            restored=restore(app.session,args.resume_room_from,state)
            (args.output/'resume.json').write_text(json.dumps(restored,indent=2))
        elif args.resume_failed_grasp_from:
            if any((args.resume_ready_release_from,args.resume_held_from,args.resume_uncertain_from,args.resume_released_from,args.resume_observation_from)):
                raise ValueError('resume_modes_are_exclusive')
            from resume_failed_grasp import restore
            restored=restore(app.session,assembled,args.resume_failed_grasp_from,state)
            (args.output/'resume.json').write_text(json.dumps(restored,indent=2))
        elif args.resume_ready_release_from:
            if any((args.resume_held_from,args.resume_uncertain_from,args.resume_released_from,args.resume_observation_from)):
                raise ValueError('resume_modes_are_exclusive')
            from resume_ready_release import restore
            restored=restore(app.session,assembled,args.resume_ready_release_from,state)
            (args.output/'resume.json').write_text(json.dumps(restored,indent=2))
        elif args.resume_uncertain_from:
            if args.resume_held_from or args.resume_released_from or args.resume_observation_from:raise ValueError('resume_modes_are_exclusive')
            from resume_uncertain import restore
            restored=restore(app.session,assembled,args.resume_uncertain_from,state)
            (args.output/'resume.json').write_text(json.dumps(restored,indent=2))
        elif args.resume_released_from:
            if args.resume_held_from or args.resume_observation_from:raise ValueError('resume_modes_are_exclusive')
            from resume_released import restore
            restored=restore(app.session,assembled,args.resume_released_from,state)
            (args.output/'resume.json').write_text(json.dumps(restored,indent=2))
        elif args.resume_held_from:
            if args.resume_observation_from:raise ValueError('resume_modes_are_exclusive')
            from resume_held import restore
            restored=restore(app.session,assembled['stages'],args.resume_held_from,state)
            (args.output/'resume.json').write_text(json.dumps(restored,indent=2))
        elif not hand_is_open(state['joints'][5]):raise ValueError('initial_hand_not_open')
        if args.resume_observation_from:
            if not args.execute:raise ValueError('resume_requires_execute')
            import math
            tucked=[0.,0.,0.,-math.pi/2,0.]
            if max(abs(a-b) for a,b in zip(state['joints'][:5],tucked))>.05:
                # Empty-hand recovery uses current table/hand geometry before
                # folding, rather than folding over the tabletop or assuming
                # the last correction returned to its original posture.
                assembled['transfer'].withdraw_source()
                runtime.prepare_navigation_from_current()
                state=runtime.state()
                (args.output/'resume-recovered-state.json').write_text(json.dumps(state,indent=2))
            from resume_observation import restore
            resumed=restore(app.session,args.resume_observation_from,state,bundle['environment'],args.resume_source_navigation)
            (args.output/'resume.json').write_text(json.dumps(resumed,indent=2))
            first_view=args.resume_observation_from.parent/'captures/identity-1/snapshot.json'
            if first_view.exists():
                joints=json.loads(first_view.read_text()).get('joints',{})
                if 'head_pan_joint' in joints and 'head_tilt_joint' in joints:
                    runtime.face_source((joints['head_pan_joint'],joints['head_tilt_joint']))
        elif not args.resume_room_from and not args.resume_failed_grasp_from and not args.resume_ready_release_from and not args.resume_held_from and not args.resume_released_from and not args.resume_uncertain_from:app.session.preflight(ready=True)
        last_progress=None
        while True:
            if interrupted:app.session.stop('operator_interrupted')
            protocol_executor.spin_once(timeout_sec=.05)
            progress=(app.session.state,app.session.pending,app.session.reason,app.executor.fault)
            if progress!=last_progress:
                snapshot=app.session.snapshot();snapshot['executor_fault']=app.executor.fault
                (args.output/'live-state.json').write_text(json.dumps(snapshot,indent=2))
                print(json.dumps(dict(event='session1_progress',state=progress[0],pending=progress[1],reason=progress[2],executor_fault=progress[3])),flush=True)
                last_progress=progress
            if app.session.state in app.session.TERMINAL and app.executor.retired:break
        report=app.session.snapshot()
        (args.output/'session-result.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report))
        return 0 if report['state']=='succeeded' else 2
    finally:
        if assembled is not None:
            app=assembled['application']
            if not app.executor.retired:
                app.session.stop('entry_exit_requested')
                while not app.executor.retired:
                    # Do not unlock a possibly moving robot simply because the
                    # Python entry encountered an exception. A failed stop proof
                    # remains visible in executor.fault and keeps ownership.
                    protocol_executor.spin_once(timeout_sec=.05)
            controller=assembled['stop_controller']
            if controller is not None:controller.close()
            if protocol_executor is not None:
                protocol_executor.remove_node(bundle['node']);protocol_executor.shutdown()
            owner.close(app,assembled['protocol'])
        else:owner.close()
        for s,handler in old.items():signal.signal(s,handler)


if __name__=='__main__':raise SystemExit(main())
