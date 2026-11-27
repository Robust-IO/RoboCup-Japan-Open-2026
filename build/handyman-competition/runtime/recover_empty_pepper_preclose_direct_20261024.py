"""Withdraw and tuck once after a reviewed preclose stop with no closure sent."""
import json,math,sys,time
from pathlib import Path
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
import numpy as np
import rclpy
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import LiveBackend
from pepper_runtime import PepperRuntime
from canned_grasp_runtime import TOPICS
from motion_speed import finite_duration
from reviewed_carry_adjustment import endpoint
from pepper_geometry import measure_near
from canned_grasp_geometry import transform_point
from task_outcome_policy import pose_matches,robot_ready

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');old=base/'pepper-native-source-auto-20261024-04'
read=lambda p:json.loads(p.read_text());stopped=read(old/'result.json');meta=read(old/'run-meta.json')
if (stopped['stage']!='close_once' or stopped['close_commands_sent'] or stopped['release_commands_sent']
        or stopped['grasp_success_verified'] or meta['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip()):
    raise ValueError('reviewed_no_close_predecessor_required')
cfg=read(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json'))
root=base/'pepper-empty-preclose-recovery-20261024-04b';root.mkdir(exist_ok=False)
report=dict(status='starting',close_commands_sent=0,release_commands_sent=0,source_reposition_requested=False)
rclpy.init();runtime=None;backend=None
with RobotLock(),(root/'journal.jsonl').open('x') as journal:
    def record(event):journal.write(json.dumps(dict(time_ns=time.time_ns(),**event))+'\n');journal.flush()
    try:
        runtime=PepperRuntime(root,'pepper-empty-recovery',record)
        backend=LiveBackend(cfg);backend.bind(root,'pepper-empty-recovery',record);backend.stage='empty_recovery'
        state=runtime.state();expected=stopped['last_fresh_robot_state']['joints']
        if not pose_matches(state['joints'],expected) or abs(state['joints'][5]-.65)>.005:
            raise ValueError('current_empty_extended_pose_not_confirmed')
        if backend.room_at(state['poses']['map'])!='bedroom':raise ValueError('source_room_not_confirmed')
        source=stopped['source_observations'][-1];s0=read(Path(source['capture'])/'snapshot.json');model=source['measurement']
        body=source['center_odom_m'];cap=transform_point(s0,model['cap_center_m'])
        sample=runtime.sample(processing_reserve_ns=0);s=sample['snapshot']
        m=measure_near(sample['rgb'],sample['depth'],s,transform_point(s,body,inverse=True),transform_point(s,cap,inverse=True),model)
        proof=runtime.fresh_measurement(s);runtime.save(sample,'before-empty-withdraw',m)
        report['actual_target_before_recovery']=m;report['current_feedback']=proof
        previous=read(base/'pepper-empty-preclose-recovery-20261024-04/empty-withdraw-from-source.json')
        if previous['command_sent']:raise ValueError('previous_withdraw_already_sent_review_required')
        topic=TOPICS[1];pub=runtime.publisher(topic)
        try:
            state=runtime.state();delta=[-.32,0.,0.];target=endpoint(state['poses']['odom'],delta)
            duration=finite_duration(runtime.profile,delta,max_translation_m=None)
            runtime.ownership(topic);pub.publish(runtime.trajectory(['odom_x','odom_y','odom_t'],target,duration))
            report['withdraw_command_sent']=True;sent=time.monotonic()
            while time.monotonic()-sent<duration+.7:
                runtime.pump(.02);runtime.state();runtime.ownership(topic)
            after=runtime.state();moved=math.dist(after['poses']['odom'][:2],state['poses']['odom'][:2])
            report['actual_withdraw_m']=moved
            if moved<.25:raise ValueError('empty_withdraw_not_observed')
        finally:runtime.node.destroy_publisher(pub)
        runtime.tuck_open();final=runtime.state();report['final_state']=final
        names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        if not robot_ready(dict(zip(names,final['joints']))):raise ValueError('empty_tucked_recovery_not_confirmed')
        report['status']='empty_withdraw_and_tuck_verified'
    except Exception as exc:report.update(status='stopped_requires_actual_state_review',reason=str(exc))
    finally:
        if backend:backend.close()
        if runtime:
            try:report['last_fresh_robot_state']=runtime.state()
            except Exception:pass
            runtime.shutdown()
        rclpy.shutdown();write_json(root/'result.json',report)
        print(json.dumps(report,allow_nan=False),flush=True)
if report['status']!='empty_withdraw_and_tuck_verified':raise SystemExit(1)
