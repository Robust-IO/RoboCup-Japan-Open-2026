"""Return the current reviewed5cm lifted can to its unchanged source table.

One5cm lowering, one opening to the actually wider0.85, one40cm retreat,
then empty-arm tuck. No new closure or transfer-success claim.
"""
import json,math,sys,time
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime,TOPICS,NAMES
from task_outcome_policy import pose_matches
from reviewed_carry_adjustment import endpoint
from motion_speed import arm_duration,finite_duration
import rclpy
base=Path('/__HANDYMAN_PACKAGE__/runtime');checkpoint=json.loads((base/'canned-deeper-grasp-verified-progress-20261024.json').read_text())
if checkpoint['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():raise ValueError('new_scene_requires_review')
root=base/'can08-return-tilted-can-to-source-20261024-01';root.mkdir(exist_ok=False)
report=dict(status='starting',source_checkpoint='D:/handyman-training-runs/canned-deeper-grasp-verified-progress-20261024.json',
    new_closure_commands_sent=0,release_commands_sent=0,release_opening_rad=.85,
    original_source_table_unchanged=True,object_upright_verified=False,automatic_action_retry=False)
with RobotLock(),(root/'journal.jsonl').open('x') as journal:
    def record(event):
        row=dict(time_ns=time.time_ns(),**event);journal.write(json.dumps(row)+'\n');journal.flush();print(json.dumps(row),flush=True)
    def stage(name):report['stage']=name;write_json(root/'result.json',report);record(dict(event='stage',stage=name))
    rclpy.init();rt=CanRuntime(root,root.name,record)
    def arm_move(goal,label):
        topic=TOPICS[2];pub=rt.publisher(topic)
        try:
            before=rt.state();duration=arm_duration(rt.profile,'arm',before['joints'],goal,holding=report['release_commands_sent']==0)
            record(dict(event='arm_intent',label=label,goal=goal,duration_s=duration))
            pub.publish(rt.trajectory(NAMES[:5],goal[:5],duration));record(dict(event='arm_sent',label=label))
            end=time.monotonic()+duration+.8
            while time.monotonic()<end:
                rt.pump(.02);current=rt.state();rt.ownership(topic)
                if abs(current['joints'][5]-before['joints'][5])>.03:raise ValueError('grip_changed_during_arm_move')
            if not pose_matches(rt.state()['joints'],goal):raise ValueError('arm_goal_not_reached:'+label)
        finally:rt.node.destroy_publisher(pub)
    try:
        sample=rt.sample();state=rt.state();old=checkpoint['last_measured_robot_state']
        if (not pose_matches(state['joints'],old['joints']) or math.dist(state['poses']['odom'][:2],old['poses']['odom'][:2])>.02
                or abs(state['poses']['odom'][2]-old['poses']['odom'][2])>.03):raise ValueError('current_lifted_scene_changed')
        report['initial_state']=state;rt.save(sample,'before-return-to-source')
        stage('lower_five_cm_to_original_table');goal=state['joints'].copy();goal[0]-=.05;arm_move(goal,'lower_to_original_source')
        rt.save(rt.sample(),'on-source-before-opening')
        stage('open_once_to_point85');topic=TOPICS[3];pub=rt.publisher(topic)
        try:
            before=rt.state();record(dict(event='open_intent',opening_rad=.85,duration_s=6.))
            pub.publish(rt.trajectory(['hand_motor_joint'],[.85],6.));report['release_commands_sent']=1;record(dict(event='open_sent'))
            end=time.monotonic()+6.8
            while time.monotonic()<end:
                rt.pump(.02);current=rt.state();rt.ownership(topic)
                if not pose_matches(current['joints'][:5]+[0.],before['joints'][:5]+[0.]):raise ValueError('arm_changed_during_open')
            if abs(rt.state()['joints'][5]-.85)>.01:raise ValueError('wide_open_not_reached')
        finally:rt.node.destroy_publisher(pub)
        stage('empty_retreat_forty_cm');topic=TOPICS[1];pub=rt.publisher(topic)
        try:
            before=rt.state();target=endpoint(before['poses']['odom'],[-.4,0.,0.],max_translation_m=None)
            duration=finite_duration(rt.profile,[-.4,0.,0.],max_translation_m=None)
            record(dict(event='empty_retreat_intent',target_odom=target,duration_s=duration))
            pub.publish(rt.trajectory(['odom_x','odom_y','odom_t'],target,duration));record(dict(event='empty_retreat_sent'))
            end=time.monotonic()+duration+1.0
            while time.monotonic()<end:
                rt.pump(.02);current=rt.state();rt.ownership(topic)
                if not pose_matches(current['joints'],before['joints']):raise ValueError('open_arm_changed_during_retreat')
            if math.dist(rt.state()['poses']['odom'][:2],target[:2])>.025:raise ValueError('retreat_not_reached')
        finally:rt.node.destroy_publisher(pub)
        stage('tuck_empty_arm');state=rt.state();goal=[state['joints'][0],0.,0.,-math.pi/2,0.,.85]
        arm_move(goal,'fold_empty');goal[0]=0.;arm_move(goal,'lower_empty')
        report['final_capture']=str(rt.save(rt.sample(),'empty-tucked-source-view'))
        report.update(status='can_returned_to_source_hand_open_robot_tucked_requires_upright_reset',
            can_no_longer_held=True,empty_arm_tucked=True,requires_user_to_restore_upright_can=True)
    except (Exception,KeyboardInterrupt) as exc:report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted')
    finally:
        try:report['last_fresh_robot_state']=rt.state()
        except Exception as exc:report['state_read_error']=str(exc)
        rt.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report),flush=True)
if not report.get('empty_arm_tucked'):raise SystemExit(1)
