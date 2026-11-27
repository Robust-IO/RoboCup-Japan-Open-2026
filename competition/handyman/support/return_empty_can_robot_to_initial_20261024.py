"""Fresh read-only audit, then return the empty robot to the reviewed start."""
import json
import math
import signal
import sys
import time
from pathlib import Path
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from can_transfer_automation import configuration,live_backend
from canned_grasp_runtime import CanRuntime
import rclpy

base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'can-empty-return-initial-20261024-09'
report=dict(status='starting',robot_commands_sent=False,grasp_commands_sent=False,release_commands_sent=False,
    navigation_parameters_changed=False,target=[1.184,0.,0.],purpose='pretest_initial_pose_reconstruction',
    not_evidence_of_an_unmoved_fresh_Unity_restart=True)
backend=None;runtime=None
def interrupted(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,interrupted)
with RobotLock() as lock:
    root.mkdir(exist_ok=False)
    write_json(root/'run-meta.json',dict(version=1,run_id=root.name,object='canned_juice',mode='live',run_root=str(root),
        started_ns=time.time_ns(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
    with (root/'journal.jsonl').open('x') as journal:
        def record(event):
            row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
        try:
            cfg=configuration();backend=live_backend(cfg);backend.bind_run(root,root.name,cfg,record,lock.stream.fileno())
            backend.perform('preflight')
            rclpy.init();runtime=CanRuntime(root,root.name,record)
            report['before_capture']=str(runtime.save(runtime.sample(),'before-return'))
            report['before_state']=runtime.state();runtime.shutdown();runtime=None;rclpy.shutdown()
            backend.stage='return_initial'
            backend.navigate(backend.start_room,'living_room',[1.184,0.,0.],backend.initial_opening,'return-initial')
            report['robot_commands_sent']=True
            rclpy.init();runtime=CanRuntime(root,root.name,record)
            report['after_capture']=str(runtime.save(runtime.sample(),'at-initial'))
            state=runtime.state();report['last_fresh_robot_state']=state
            pose=state['poses']['map'];error=math.hypot(pose[0]-1.184,pose[1])
            yaw=abs(math.atan2(math.sin(pose[2]),math.cos(pose[2])))
            if error>.061 or yaw>.081:raise ValueError('current_base_not_at_reviewed_start')
            report.update(status='empty_robot_returned_to_initial_for_new_trial',initial_position_error_m=error,
                initial_yaw_error_rad=yaw)
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
        finally:
            if backend:backend.close()
            if runtime:runtime.shutdown()
            if rclpy.ok():rclpy.shutdown()
            write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if report['status']!='empty_robot_returned_to_initial_for_new_trial':raise SystemExit(1)
