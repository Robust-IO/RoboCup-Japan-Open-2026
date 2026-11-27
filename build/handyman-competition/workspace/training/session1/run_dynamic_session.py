"""Opt-in live instruction-driven task runner; default only validates inputs."""
import argparse,json,signal,uuid
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path);p.add_argument('--execute',action='store_true');p.add_argument('--prepare-initial-empty',action='store_true');a=p.parse_args()
    cfg=json.loads(a.config.read_text());root=Path(__file__).resolve().parents[3]
    if not a.execute:
        from live_config import validate
        validate(cfg);print(json.dumps({'configuration_valid':True,'robot_commands_sent':False}));return 0
    if a.output is None:p.error('--output required')
    from runtime_resources import RuntimeResources
    from motion_stop import MotionGate,StopController
    from stage_bindings import StageBindings
    from candidate_selection import load_selector
    from shared_turn_guard import install
    from dynamic_bindings import DynamicBindings
    from ros_protocol import bind_application
    owner=RuntimeResources(a.config,root/'runtime/autograsp-v1/replay-inputs/hand.json',a.output)
    bundle=owner.open(prepare_initial_empty=a.prepare_initial_empty);app=protocol=executor=None;interrupted=False
    def stop(*_):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        rt=bundle['runtime'];motion=bundle['motion_backend'];gate=MotionGate();gate.install(rt,motion);controller=StopController(rt,motion,gate,a.output)
        stages=StageBindings(rt,motion,bundle['grasp_backend'],bundle['config'],bundle['environment'],a.output,candidate_selector=load_selector(bundle['config'],bundle['semantic']))
        install(stages);bindings=DynamicBindings(bundle,stages)
        app,protocol=bind_application(bundle['node'],'dynamic-'+uuid.uuid4().hex,bindings.handlers(),controller.request_stop,profile={'schema':'handyman_dynamic_instructions_v1','layout':cfg['layout']})
        controller.bind(app)
        from rclpy.executors import SingleThreadedExecutor
        executor=SingleThreadedExecutor();executor.add_node(bundle['node'])
        rt.sample(profile='state');state=rt.state()
        if motion.room_at(state['poses']['map']) is None:raise ValueError('initial_room_not_localized')
        if state['joints'][5]<.6:raise ValueError('empty_hand_start_required')
        app.session.preflight(ready=True);last=None
        while True:
            if interrupted:app.session.stop('operator_cancelled')
            executor.spin_once(timeout_sec=.05)
            snapshot=app.session.snapshot();progress=(snapshot['state'],snapshot['reason'])
            if progress!=last:
                (a.output/'live-state.json').write_text(json.dumps(snapshot,indent=2));print(json.dumps({'event':'dynamic_progress','state':progress[0],'reason':progress[1]}),flush=True);last=progress
            if app.session.state in app.session.TERMINAL and app.executor.retired:break
        report=app.session.snapshot();(a.output/'session-result.json').write_text(json.dumps(report,indent=2));return 0 if report['state']=='succeeded' else 2
    finally:
        if app is not None and not app.executor.retired:
            app.session.stop('runner_exit')
            while not app.executor.retired:executor.spin_once(timeout_sec=.05)
        if executor is not None:executor.remove_node(bundle['node']);executor.shutdown()
        owner.close(app,protocol)
if __name__=='__main__':raise SystemExit(main())
