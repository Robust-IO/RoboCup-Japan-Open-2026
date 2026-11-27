"""Owned live resources. open() is explicit; importing this module has no effects.

No motion, protocol readiness or task execution is started by this lifecycle.
Only a fresh standalone process with the existing domain71 setup may open it.
"""
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from threading import Lock
import json
import os
import time
import uuid


def dependencies():
    import rclpy
    import yaml
    from rclpy.signals import SignalHandlerOptions
    from apple_transfer_automation import RobotLock
    from apple_worker_client import Worker
    from thin_box_clearance import ThinBoxClearance
    from outcome_transfer_runtime import OutcomeBackend
    from grasp_runtime.autograsp.ros_backend import RosBackend,runtime_class
    from grasp_runtime.autograsp.planner import Hand,Policy
    return SimpleNamespace(ros=rclpy,init_ros=lambda:rclpy.init(signal_handler_options=SignalHandlerOptions.NO),yaml=yaml,lock=RobotLock,worker=Worker,
        physical=ThinBoxClearance,motion=OutcomeBackend,grasp=RosBackend,
        runtime_class=runtime_class,hand=Hand,policy=Policy)


class RuntimeResources:
    def __init__(self,config_path,hand_path,output,*,dependency_factory=dependencies):
        self.config_path=Path(config_path);self.hand_path=Path(hand_path);self.output=Path(output)
        self.dependency_factory=dependency_factory;self.stack=None;self.bundle=None;self.used=False

    def open(self,*,prepare_initial_empty=False):
        opened_at=time.monotonic()
        if self.used:raise ValueError('resource_owner_is_single_use')
        if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise ValueError('existing_domain71_localhost_environment_required')
        if not any(self.output.resolve().is_relative_to(base) for base in (Path('/__HANDYMAN_PACKAGE__/runs'),Path('/__HANDYMAN_PACKAGE__/runtime'))):
            raise ValueError('new_output_directory_on_D_required')
        self.used=True
        cfg=json.loads(self.config_path.read_text())
        from live_config import validate
        validate(cfg)
        d=self.dependency_factory()
        if d.ros.ok():raise ValueError('standalone_ros_context_required')
        semantic=json.loads(Path(cfg['semantic_map']).read_text())
        environment=d.yaml.safe_load(Path(cfg['environment']).read_text())
        table=next(t for t in semantic['destinations'] if t['id']==cfg['source_table_id'])
        hand=d.hand(json.loads(self.hand_path.read_text()))
        rules=json.loads((Path(__file__).parent/'grasp_runtime/grasp_strategy_policy.json').read_text())
        settings=dict(rules['default']);settings['allowed_strategies']=tuple(settings['allowed_strategies'])
        policy=d.policy(**settings)
        stack=ExitStack()
        try:
            stack.enter_context(d.lock())
            self.output.mkdir(parents=True,exist_ok=False)
            log=stack.enter_context((self.output/'journal.jsonl').open('x'))
            mutex=Lock()
            def record(event):
                with mutex:
                    log.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');log.flush()
            getattr(d,'init_ros',d.ros.init)();stack.callback(d.ros.shutdown)
            runtime=d.runtime_class()(self.output,'session1-'+uuid.uuid4().hex,record)
            stack.callback(runtime.shutdown)
            previous=os.environ.get('HANDYMAN_RESIDENT_MODEL')
            os.environ['HANDYMAN_RESIDENT_MODEL']='1'
            def restore_env():
                if previous is None:os.environ.pop('HANDYMAN_RESIDENT_MODEL',None)
                else:os.environ['HANDYMAN_RESIDENT_MODEL']=previous
            stack.callback(restore_env)
            model_at=time.monotonic()
            if prepare_initial_empty:
                from startup_parallel import prepare_with_model
                worker=prepare_with_model(
                    lambda:d.worker(cfg['model_python'],cfg['registry'],self.output/'model.log',imgsz=960),
                    runtime,self.output)
            else:
                worker=d.worker(cfg['model_python'],cfg['registry'],self.output/'model.log',imgsz=960)
            record(dict(event='startup_model_and_hand_ready',elapsed_s=time.monotonic()-model_at,
                        resources_elapsed_s=time.monotonic()-opened_at,parallel=prepare_initial_empty))
            stack.callback(worker.close)
            physical=d.physical().hand
            grasp=d.grasp(runtime,worker,table,hand,physical,record,policy=policy)
            grasp.source_constrained=bool(cfg.get('source_table_id') and cfg.get('task',{}).get('source_room'))
            grasp.use_family_templates=True
            from object_grasp_profiles import apply as apply_object_grasp_profile
            apply_object_grasp_profile(grasp,cfg['task']['object'])
            motion=d.motion(cfg);motion.bind(self.output,'session1-'+uuid.uuid4().hex,record)
            motion.stage='session1'
            motion.observation_runtime=runtime
            node=d.ros.create_node('handyman_session1_protocol');stack.callback(node.destroy_node)
            self.bundle=dict(node=node,runtime=runtime,worker=worker,grasp_backend=grasp,
                             motion_backend=motion,config=cfg,environment=environment,semantic=semantic,output=self.output)
            self.stack=stack
            (self.output/'config.snapshot.json').write_text(json.dumps(cfg,indent=2))
            record(dict(event='session1_resources_opened',task_started=False))
            return self.bundle
        except BaseException:
            stack.close();self.stack=None;self.bundle=None
            raise

    def close(self,application=None,protocol=None):
        if application is not None and not application.executor.retired:
            raise RuntimeError('confirm_motion_retired_before_releasing_control_lock')
        if self.bundle is not None:
            child=getattr(self.bundle['motion_backend'],'active_child',None)
            if child is not None and child.poll() is None:
                raise RuntimeError('active_child_must_retire_before_resource_cleanup')
        if protocol is not None:protocol.close()
        if application is not None:application.close()
        if self.stack is not None:
            try:self.stack.close()
            finally:self.stack=None;self.bundle=None
