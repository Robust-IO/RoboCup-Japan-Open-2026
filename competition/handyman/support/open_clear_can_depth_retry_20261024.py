"""User-requested open, straight retreat and empty-arm reset; no grasp."""
import json
import math
import sys
import time
from pathlib import Path
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime,NAMES,TOPICS
from reviewed_carry_adjustment import endpoint
from motion_speed import finite_duration
import rclpy

base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'can-centerline-open-retreat-20261024-02'
report=dict(status='starting',robot_commands_sent=False,close_commands_sent=0,
    requested_retreat_m=.45,automatic_action_retry=False)
runtime=None
with RobotLock():
    root.mkdir(exist_ok=False)
    with (root/'journal.jsonl').open('x') as journal:
        def record(event):
            row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
        def move(topic,names,target,duration,label):
            pub=runtime.publisher(topic)
            try:
                runtime.ownership(topic);record(dict(event='finite_command_once',label=label,target=target,duration_s=duration))
                pub.publish(runtime.trajectory(names,target,duration));report['robot_commands_sent']=True
                sent=time.monotonic();state=None
                while time.monotonic()-sent<duration+.7:
                    runtime.pump(.02);state=runtime.state();runtime.ownership(topic)
                if topic==TOPICS[2] and max(abs(a-b) for a,b in zip(state['joints'][:5],target))>.005:
                    raise ValueError('arm_endpoint_not_observed:'+label)
                if topic==TOPICS[3] and abs(state['joints'][5]-target[0])>.008:
                    raise ValueError('opening_endpoint_not_observed:'+label)
                return state
            finally:runtime.node.destroy_publisher(pub)
        try:
            rclpy.init();runtime=CanRuntime(root,root.name,record)
            sample=runtime.sample();initial=runtime.state();report['initial_state']=initial
            if abs(initial['joints'][5]-.65)>.015:raise ValueError('current_hand_not_at_audited_empty_opening')
            runtime.save(sample,'before-opening')
            report['stage']='open_empty_hand';write_json(root/'result.json',report)
            move(TOPICS[3],[NAMES[5]],[.85],1.5,'open_before_straight_retreat')
            report['stage']='straight_retreat';write_json(root/'result.json',report)
            state=runtime.state();start=state['poses']['odom'];delta=[-.45,0.,0.]
            target=endpoint(start,delta,max_translation_m=None)
            state=move(TOPICS[1],['odom_x','odom_y','odom_t'],target,
                finite_duration(runtime.profile,delta,max_translation_m=None),'clear_straight_backward')
            report['actual_retreat_m']=math.dist(start[:2],state['poses']['odom'][:2])
            if math.dist(target[:2],state['poses']['odom'][:2])>.025:raise ValueError('retreat_endpoint_not_observed')
            report['stage']='tuck_empty_arm';write_json(root/'result.json',report)
            arm=state['joints'][:5].copy()
            for index,value,label in ((1,0.,'fold_empty_elbow'),(3,-math.pi/2,'restore_wrist'),(0,0.,'lower_empty_arm')):
                arm[index]=value;move(TOPICS[2],NAMES[:5],arm.copy(),1.5,label)
            move(TOPICS[3],[NAMES[5]],[.65],1.2,'normalize_empty_open_hand')
            report['final_capture']=str(runtime.save(runtime.sample(),'empty_tucked_after_retreat'))
            report.update(status='open_retreat_empty_reset_verified',hand_empty=True,arm_tucked=True)
        except (Exception,KeyboardInterrupt) as exc:
            report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
        finally:
            if runtime:
                try:report['last_fresh_robot_state']=runtime.state()
                except Exception as exc:report['state_read_error']=str(exc)
                runtime.shutdown()
            if rclpy.ok():rclpy.shutdown()
            write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if report['status']!='open_retreat_empty_reset_verified':raise SystemExit(1)
