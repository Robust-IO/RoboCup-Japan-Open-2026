"""Empty return to initial pose after run06 stopped before any arm command."""
import json,math,subprocess,sys,time
from pathlib import Path
CODE=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import LiveBackend
from canned_grasp_runtime import CanRuntime,TOPICS
from reviewed_carry_adjustment import endpoint
from motion_speed import finite_duration
from task_outcome_policy import pose_matches
import rclpy
base=Path('/__HANDYMAN_PACKAGE__/runtime');prior=base/'can-auto-live-20261023-06';read=lambda p:json.loads(p.read_text())
grasp=read(prior/'grasp/result.json');prepare=read(prior/'grasp/prepare/result.json')
if (grasp['stage']!='prepare_horizontal' or grasp.get('close') or prepare.get('events')
        or prepare.get('grasp_closure_commanded') or prepare.get('base_anchor') is not None):raise ValueError('previous_no_arm_command_proof_required')
startup=read(base/'can-startup-live-verification-20261023-01/result.json')
if startup['status']!='five_current_can_startup_windows_verified_without_motion':raise ValueError('actual_startup_verification_required')
root=base/'can06-return-initial-20261023-01';root.mkdir(exist_ok=False)
cfg=read(CODE/'config/apple_transfer_auto_v1.json');report=dict(status='starting',hand_commands_sent=0,arm_commands_sent=0,source_can_ungrasped=True)
backend=None;runtime=None
with RobotLock(),(root/'journal.jsonl').open('x') as journal:
    def record(x):
        row=dict(time_ns=time.time_ns(),**x);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    try:
        path=root/'preflight.json'
        p=subprocess.run([sys.executable,str(CODE/'inspect_transfer_ready.py'),'--output',str(path),'--efficient-transfer'],timeout=25)
        ready=read(path)
        if p.returncode or not ready.get('ready') or abs(ready['joints']['hand_motor_joint'])>.005:raise ValueError('empty_tucked_robot_required')
        backend=LiveBackend(cfg);backend.bind(root,root.name,record);backend.stage='empty_return_to_initial'
        if backend.room_at(ready['base_map_pose'])!='bedroom':raise ValueError('source_bedroom_required')
        report['initial_state']=ready;write_json(root/'result.json',report)
        backend.navigate('bedroom','living_room',[1.184,0.,0.],0.,'return-navigation');backend.close();backend=None
        rclpy.init();runtime=CanRuntime(root,'can06-empty-return-initial',record)
        runtime.sample(processing_reserve_ns=0);state=runtime.state();mapped=state['poses']['map']
        if not pose_matches(state['joints'],[0.,0.,0.,-math.pi/2,0.,0.]):raise ValueError('arm_or_hand_changed_during_empty_return')
        # Preserve Nav2 tolerances. One measured finite final alignment sets the
        # test's actual initial pose; it is not a stricter navigation goal gate.
        dx=1.184-mapped[0];dy=-mapped[1];yaw=math.atan2(math.sin(-mapped[2]),math.cos(-mapped[2]))
        if math.hypot(dx,dy)>.1 or abs(yaw)>.15:raise ValueError('initial_navigation_outside_reviewed_alignment')
        delta=[math.cos(mapped[2])*dx+math.sin(mapped[2])*dy,-math.sin(mapped[2])*dx+math.cos(mapped[2])*dy,yaw]
        target=endpoint(state['poses']['odom'],delta);duration=finite_duration(runtime.profile,delta)
        topic=TOPICS[1];pub=runtime.publisher(topic)
        try:
            record(dict(event='empty_initial_alignment_intent',delta=delta,target_odom=target,duration_s=duration))
            pub.publish(runtime.trajectory(['odom_x','odom_y','odom_t'],target,duration));record(dict(event='empty_initial_alignment_sent'))
            until=time.monotonic()+duration+.8
            while time.monotonic()<until:
                runtime.pump(.02);actual=runtime.state();runtime.ownership(topic)
                if not pose_matches(actual['joints'],state['joints']):raise ValueError('empty_arm_changed_during_initial_alignment')
        finally:runtime.node.destroy_publisher(pub)
        final=runtime.state();actual=final['poses']['map']
        if math.dist(actual[:2],[1.184,0.])>.02 or abs(actual[2])>.03:raise ValueError('initial_pose_not_verified')
        report.update(status='empty_robot_back_at_initial_pose_verified',final_state=final,robot_commands_sent=True,
            navigation_parameters_changed=False,hand_commands_sent=0,arm_commands_sent=0)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped_requires_actual_state_review',reason=str(exc))
    finally:
        if backend:backend.close()
        if runtime:runtime.shutdown()
        if rclpy.ok():rclpy.shutdown()
        write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if report['status']!='empty_robot_back_at_initial_pose_verified':raise SystemExit(1)
