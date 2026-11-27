"""Empty hand only: restore known0.65 opening after releasing and withdrawing."""
import json,math,sys,time
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime,TOPICS
from task_outcome_policy import pose_matches
import rclpy
base=Path('/__HANDYMAN_PACKAGE__/runtime');previous=json.loads((base/'can08-return-tilted-can-to-source-20261024-01/result.json').read_text())
if not previous.get('can_no_longer_held') or not previous.get('empty_arm_tucked'):raise ValueError('actual_empty_retreat_required')
root=base/'can08-empty-hand-normalized-20261024-01';root.mkdir(exist_ok=False)
with RobotLock():
    rclpy.init();rt=CanRuntime(root,root.name,lambda event:None);pub=None
    report=dict(status='starting',grasp_attempted=False,object_held=False)
    try:
        sample=rt.sample();current=rt.state();old=previous['last_fresh_robot_state']
        if (not pose_matches(current['joints'],old['joints']) or math.dist(current['poses']['odom'][:2],old['poses']['odom'][:2])>.02):
            raise ValueError('empty_robot_changed_since_withdrawal')
        rt.save(sample,'before-empty-normalization');topic=TOPICS[3];pub=rt.publisher(topic)
        pub.publish(rt.trajectory(['hand_motor_joint'],[.65],6.));report['one_empty_hand_command_sent']=True
        end=time.monotonic()+6.8
        while time.monotonic()<end:
            rt.pump(.02);state=rt.state();rt.ownership(topic)
            if not pose_matches(state['joints'][:5]+[0.],current['joints'][:5]+[0.]):raise ValueError('tucked_arm_changed')
        state=rt.state()
        if abs(state['joints'][5]-.65)>.01:raise ValueError('empty_hand_not_normalized')
        rt.node.destroy_publisher(pub);pub=None
        report.update(status='empty_tucked_hand_normalized',current_state=state,
            capture=str(rt.save(rt.sample(),'empty-normalized-current')))
    except Exception as exc:report.update(status='stopped_requires_actual_state_review',reason=str(exc))
    finally:
        if pub:rt.node.destroy_publisher(pub)
        rt.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if report['status']!='empty_tucked_hand_normalized':raise SystemExit(1)
