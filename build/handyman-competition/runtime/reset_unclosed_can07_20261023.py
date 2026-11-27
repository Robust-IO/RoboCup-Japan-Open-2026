"""One explicit empty-arm retreat/tuck after audited run07 pre-closure stop."""
import json,math,sys,time
from pathlib import Path
import numpy as np
CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime,TOPICS,NAMES
from canned_grasp_geometry import fresh
from task_outcome_policy import pose_matches
from reviewed_carry_adjustment import endpoint
from motion_speed import finite_duration,arm_duration
import rclpy
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');prior_root=base/'can-auto-live-20261023-07'
prior=json.loads((prior_root/'grasp/result.json').read_text());meta=json.loads((prior_root/'run-meta.json').read_text())
events=[json.loads(x) for x in (prior_root/'grasp/journal.jsonl').read_text().splitlines()]
if (prior['stage']!='measured_approach' or prior.get('close') or prior.get('lift')
        or (prior_root/'grasp/closure-contact.json').exists()
        or any(x.get('event') in ('can_closure_intent','can_closure_sent') for x in events)):
    raise ValueError('prior_open_no_closure_required')
if meta['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():raise ValueError('boot_changed')
root=base/'can07-empty-retreat-tuck-20261023-01';root.mkdir(exist_ok=False)
report=dict(status='starting',predecessor=str(prior_root),no_previous_closure=True,
    hand_commands_sent=0,grasp_or_placement_attempted=False,old_run_verdict_preserved=True)
with RobotLock(),(root/'journal.jsonl').open('x') as journal:
    def record(x):
        row=dict(time_ns=time.time_ns(),**x);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    rclpy.init();rt=CanRuntime(root,'can07-empty-reset',record)
    try:
        sample=rt.sample();current=rt.state();old=prior['last_fresh_robot_state']
        if not pose_matches(current['joints'],old['joints']) or abs(current['joints'][5]-.65)>.02:
            raise ValueError('current_open_arm_changed')
        if (math.dist(current['poses']['odom'][:2],old['poses']['odom'][:2])>.02
                or abs(current['poses']['odom'][2]-old['poses']['odom'][2])>.03):raise ValueError('current_base_changed')
        rt.save(sample,'before-empty-retreat');report['initial_state']=current
        topic=TOPICS[1];pub=rt.publisher(topic)
        try:
            current=rt.state();delta=[-.40,0.,0.];target=endpoint(current['poses']['odom'],delta,max_translation_m=None)
            duration=finite_duration(rt.profile,delta,max_translation_m=None)
            record(dict(event='empty_retreat_intent',target_odom=target,duration_s=duration))
            pub.publish(rt.trajectory(['odom_x','odom_y','odom_t'],target,duration));record(dict(event='empty_retreat_sent'))
            deadline=time.monotonic()+duration+.8
            while time.monotonic()<deadline:
                rt.pump(.02);state=rt.state();rt.ownership(topic)
                if not pose_matches(state['joints'],current['joints']):raise ValueError('arm_moved_during_empty_retreat')
            if math.dist(rt.state()['poses']['odom'][:2],target[:2])>.025:raise ValueError('empty_retreat_not_reached')
        finally:rt.node.destroy_publisher(pub)
        topic=TOPICS[2];pub=rt.publisher(topic)
        try:
            for label,goal in [('fold',[current['joints'][0],0.,0.,-math.pi/2,0.,.65]),('lower',[0.,0.,0.,-math.pi/2,0.,.65])]:
                before=rt.state();duration=arm_duration(rt.profile,'arm',before['joints'],goal)
                record(dict(event='empty_arm_intent',step=label,goal=goal,duration_s=duration))
                pub.publish(rt.trajectory(NAMES[:5],goal[:5],duration));record(dict(event='empty_arm_sent',step=label))
                deadline=time.monotonic()+duration+.8
                while time.monotonic()<deadline:
                    rt.pump(.02);state=rt.state();rt.ownership(topic)
                    if abs(state['joints'][5]-.65)>.02:raise ValueError('empty_hand_opening_changed')
                if not pose_matches(rt.state()['joints'],goal):raise ValueError('empty_arm_goal_not_reached:'+label)
        finally:rt.node.destroy_publisher(pub)
        report.update(status='empty_retreat_and_tuck_verified',final_state=rt.state(),robot_commands_sent=True)
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='stopped_requires_actual_state_review',reason=str(exc))
        try:report['last_fresh_robot_state']=rt.state()
        except Exception as error:report['state_error']=str(error)
    finally:
        rt.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if report['status']!='empty_retreat_and_tuck_verified':raise SystemExit(1)
