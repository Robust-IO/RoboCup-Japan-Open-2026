"""One new user-directed close after the separately verified4cm advance.

Current manual grasp trial, not a whole-run continuation or proof of holding.
No arm, base, lift or automatic reopening is performed.
"""
import json,math,sys,time
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime,TOPICS
from gripper_close_gate import CloseGate
from task_outcome_policy import pose_matches
import rclpy
base=Path('/__HANDYMAN_PACKAGE__/runtime');forward=base/'can08-forward-four-cm-20261024-01'
prior=json.loads((forward/'result.json').read_text())
if prior['status']!='one_four_cm_forward_completed_requires_new_grasp_review' or not prior['command_sent']:
    raise ValueError('actual_four_cm_advance_required')
root=base/'can08-deeper-close-20261024-01';root.mkdir(exist_ok=False)
report=dict(status='starting',prior_forward=str(forward),grasp_success_verified=False,
    physical_transfer_verified=False,close_commands_sent=0,automatic_action_retry=False,hand_released_by_cleanup=False)
with RobotLock(),(root/'journal.jsonl').open('x') as journal:
    def record(x):
        row=dict(time_ns=time.time_ns(),**x);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    rclpy.init();rt=CanRuntime(root,root.name,record);pub=None
    try:
        sample=rt.sample();state=rt.state();old=prior['final_state']
        if (not pose_matches(state['joints'],old['joints']) or abs(state['joints'][5]-.85)>.01
                or math.dist(state['poses']['odom'][:2],old['poses']['odom'][:2])>.02
                or abs(state['poses']['odom'][2]-old['poses']['odom'][2])>.03):raise ValueError('audited_deeper_open_pose_changed')
        path=rt.save(sample,'before-close');report['before_capture']=str(path);report['initial_state']=state
        topic=TOPICS[3];pub=rt.publisher(topic);state=rt.state();arm=state['joints'][:5];gate=CloseGate(state['joints'][5],6.)
        record(dict(event='new_deeper_close_intent',initial_opening=state['joints'][5],duration_s=6.))
        pub.publish(rt.trajectory(['hand_motor_joint'],[0.],6.));report['close_commands_sent']=1;record(dict(event='new_deeper_close_sent'))
        sent=time.monotonic();last=state['stamp_ns'];decision=None
        while time.monotonic()-sent<11:
            rt.pump(.02);current=rt.state();rt.ownership(topic)
            if current['stamp_ns']<=last:continue
            last=current['stamp_ns'];decision=gate.observe(time.monotonic()-sent,last,current['joints'][5],
                max(abs(a-b) for a,b in zip(arm,current['joints'][:5])))
            if decision is not None:break
        rt.node.destroy_publisher(pub);pub=None
        captures=[str(rt.save(rt.sample(),f'after-close-{i:02d}')) for i in range(3)]
        report.update(status='deeper_close_contact_candidate_requires_lift_verification'
            if decision and decision['state']=='contact_candidate' else 'deeper_close_not_confirmed',
            closure_evidence=decision,captures=captures,final_state=rt.state())
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped_requires_actual_state_review',reason=str(exc))
    finally:
        if pub:rt.node.destroy_publisher(pub)
        rt.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if report['status']!='deeper_close_contact_candidate_requires_lift_verification':raise SystemExit(1)
