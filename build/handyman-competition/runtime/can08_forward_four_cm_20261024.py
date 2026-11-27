"""One user-directed4cm advance from audited open hand, no closure or lift."""
import json,math,sys,time
from pathlib import Path
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime,TOPICS
from reviewed_carry_adjustment import endpoint
from motion_speed import finite_duration
from task_outcome_policy import pose_matches
import rclpy
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');old=base/'can08-deeper-seat-live-20261024-01/grasp'
prior=json.loads((old/'result.json').read_text());events=[json.loads(x) for x in (old/'journal.jsonl').read_text().splitlines()]
if prior['stage']!='one_wider_open_to_clear_fingers' or any(x['event'] in ('deeper_seat_forward_sent','can_closure_sent') for x in events):
    raise ValueError('prior_forward_or_close_must_not_be_repeated')
root=base/'can08-forward-four-cm-20261024-01';root.mkdir(exist_ok=False)
report=dict(status='starting',requested_delta_m=[.04,0.,0.],command_sent=False,hand_or_arm_commands_sent=False,
    previous_open_replayed=False,grasp_success_verified=False,automatic_action_retry=False)
with RobotLock(),(root/'journal.jsonl').open('x') as journal:
    def record(x):
        row=dict(time_ns=time.time_ns(),**x);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    rclpy.init();rt=CanRuntime(root,root.name,record);pub=None
    try:
        sample=rt.sample();state=rt.state();before=prior['last_fresh_robot_state']
        if (not pose_matches(state['joints'],before['joints']) or abs(state['joints'][5]-.85)>.01
                or math.dist(state['poses']['odom'][:2],before['poses']['odom'][:2])>.02
                or abs(state['poses']['odom'][2]-before['poses']['odom'][2])>.03):raise ValueError('audited_current_open_pose_changed')
        rt.save(sample,'before');report['initial_state']=state
        topic=TOPICS[1];pub=rt.publisher(topic);state=rt.state()
        target=endpoint(state['poses']['odom'],[.04,0.,0.]);duration=finite_duration(rt.profile,[.04,0.,0.])
        record(dict(event='four_cm_forward_intent',target_odom=target,duration_s=duration))
        pub.publish(rt.trajectory(['odom_x','odom_y','odom_t'],target,duration));report['command_sent']=True;record(dict(event='four_cm_forward_sent'))
        until=time.monotonic()+duration+1.2
        while time.monotonic()<until:
            rt.pump(.02);actual=rt.state();rt.ownership(topic)
            if not pose_matches(actual['joints'],state['joints']):raise ValueError('open_arm_changed_during_forward')
        rt.node.destroy_publisher(pub);pub=None
        actual=rt.state()
        if math.dist(actual['poses']['odom'][:2],target[:2])>.015:raise ValueError('forward_not_completed')
        captures=[]
        for i in range(2):captures.append(str(rt.save(rt.sample(),f'after-{i:02d}')))
        report.update(status='one_four_cm_forward_completed_requires_new_grasp_review',final_state=actual,captures=captures)
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped_requires_actual_state_review',reason=str(exc))
    finally:
        if pub:rt.node.destroy_publisher(pub)
        rt.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if report['status']!='one_four_cm_forward_completed_requires_new_grasp_review':raise SystemExit(1)
