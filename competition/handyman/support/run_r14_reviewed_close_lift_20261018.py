"""New live near-target validation, then ONE corrected close/lift. No old replay."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');base=Path('/__HANDYMAN_PACKAGE__/runtime')
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,check_inputs,write_json
from apple_transfer_auto_plan import fresh_snapshot,map_pose
from apple_approach_lease import read_latest
from apple_transfer_auto_backend import stop_group
from hand_camera_ray import rotation
from latest_capture_index import latest


def current_near_readiness(output,previous):
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer,TransformListener
    from rcl_interfaces.srv import GetParameters
    from lifecycle_msgs.srv import GetState
    from fresh_feedback_wait import wait_for_fresh
    from inspect_transfer_ready import validate_sample_times
    rclpy.init();node=rclpy.create_node('r14_near_continuation_readonly');buf=Buffer();listener=TransformListener(buf,node)
    data={};sub=node.create_subscription(JointState,'/hsrb/joint_states',lambda m:data.update(joints=m),qos_profile_sensor_data)
    report=dict(ready=False,robot_commands_sent=False)
    def call(client,request):
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('read_service_missing')
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done():raise RuntimeError('read_service_timeout')
        return future.result()
    def feedback():
        for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
        if 'joints' not in data:raise RuntimeError('stale_joints')
        mapped=buf.lookup_transform('map','base_footprint',Time())
        odom=buf.lookup_transform('odom','base_footprint',Time())
        msg=data['joints'];now=node.get_clock().now().nanoseconds
        report['feedback_ages_s']=validate_sample_times(Time.from_msg(msg.header.stamp).nanoseconds,Time.from_msg(mapped.header.stamp).nanoseconds,now)
        if not -.02<=(now-Time.from_msg(odom.header.stamp).nanoseconds)/1e9<=.4:raise RuntimeError('stale_base_tf')
        return msg,mapped,odom
    def pose(tf):
        t=tf.transform.translation;q=tf.transform.rotation
        return [t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    try:
        end=time.monotonic()+4
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
        req=GetParameters.Request();req.names=['general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','FollowPath.xy_goal_tolerance']
        tolerances=[v.double_value for v in call(node.create_client(GetParameters,'/controller_server/get_parameters'),req).values]
        state=call(node.create_client(GetState,'/controller_server/get_state'),GetState.Request()).current_state.id
        message,mapped,odom=wait_for_fresh(feedback,lambda dt:rclpy.spin_once(node,timeout_sec=dt))
        joints=dict(zip(message.name,message.position))
        topics=['/hsrb/'+n for n in ('command_velocity','omni_base_controller/command','arm_trajectory_controller/command','gripper_controller/command','head_trajectory_controller/command')]
        publishers={topic:node.count_publishers(topic) for topic in topics}
        subscribers={topic:node.count_subscribers(topic) for topic in topics[1:]}
        current=pose(mapped);current_odom=pose(odom)
        report.update(joints=joints,base_map_pose=current,base_odom_pose=current_odom,
            command_publishers=publishers,command_subscribers=subscribers,controller_state=state,navigation_tolerances=tolerances,
            checked_ns=node.get_clock().now().nanoseconds)
        if any(publishers.values()) or any(v!=1 for v in subscribers.values()) or state not in (1,2):raise ValueError('current_control_not_exclusive_or_idle')
        if len(tolerances)!=3 or any(abs(x-y)>1e-8 for x,y in zip(tolerances,[.06,.08,.06])):raise ValueError('navigation_parameters_changed_no_writes')
        prior=previous['joints']
        for key in ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint','head_pan_joint','head_tilt_joint'):
            if key not in joints or not math.isfinite(joints[key]) or abs(joints[key]-prior[key])>.006:raise ValueError('posture_changed_since_stop:'+key)
        if not .24<=joints['arm_lift_joint']<=.30 or abs(joints['hand_motor_joint']-.65)>.005:raise ValueError('prepared_open_arm_required')
        for current_pose,transform in ((current,previous['tracking_transforms']['map_from_base_at_depth']),
                                        (current_odom,previous['tracking_transforms']['odom_from_base_at_depth'])):
            rot=rotation(transform['quaternion']);yaw=math.atan2(rot[1,0],rot[0,0])
            if (math.dist(current_pose[:2],transform['translation'][:2])>.015
                or abs(math.atan2(math.sin(current_pose[2]-yaw),math.cos(current_pose[2]-yaw)))>.02):
                raise ValueError('scene_or_robot_changed_no_old_pose_reuse')
        report.update(ready=True,holding_object=False,prepared_open_arm=True,current_scene_continuity_checked=True)
    except Exception as exc:report['reason']=str(exc)
    finally:
        write_json(output,report);node.destroy_node();rclpy.try_shutdown()
    if report['ready'] is not True:raise ValueError('near_readiness:'+report.get('reason','not_ready'))
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',action='store_true');args=parser.parse_args()
    if not args.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('explicit_domain71_run_required')
    out=base/'apple-r14-reviewed-close-lift-20261018-01'
    with RobotLock():
        out.mkdir(exist_ok=False)
        cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
        deployed=json.loads((base/'apple-close-handoff-r14-20261018-deployment.json').read_text())
        if deployed['status']!='deployed_tests_passed':raise ValueError('tested_r14_required')
        for name,digest in deployed['hashes'].items():
            if hashlib.sha256((code/name).read_bytes()).hexdigest()!=digest:raise ValueError('r14_code_changed:'+name)
        old=json.loads((base/'apple-capture-budget-live-progress-20261018.json').read_text())
        previous=json.loads((Path(old['current_robot']['capture'])/'snapshot.json').read_text())
        task='apple-r14-close-'+uuid.uuid4().hex;epoch=task+'-observe';recording=out/'observe'
        report=dict(status='starting',task_id=task,epoch=epoch,recording=str(recording),
            old_actions_replayed=False,old_identity_restored=False,base_approach_commands_sent=0,
            whole_transfer_verified=False,protocol_messages_sent=False,automatic_retry=False)
        observer=None;child=None;observer_log=None;rows=[]
        def save(status,**fields):
            report.update(status=status,**fields);write_json(out/'status.json',report)
            print(json.dumps(dict(status=status,**fields)),flush=True)
        try:
            write_json(out/'input-files.json',check_inputs(cfg))
            ready=current_near_readiness(out/'before.json',previous)
            save('new_readonly_observation',current_pose=ready['base_map_pose'],arm_lift_m=ready['joints']['arm_lift_joint'])
            command=[sys.executable,str(code/'capture_grasp_scene.py'),str(recording),'--hand-geometry','--map-geometry',
                '--tracking-epoch',epoch,'--duration','240','--interval','.1','--compact-progress',
                '--inference-python',cfg['model_python'],'--registry',cfg['registry'],'--inference-imgsz','960',
                '--environment-yaml',cfg['environment'],'--expected-layout','LayoutA','--target-room','bedroom',
                '--identity-tracking-task',task,'--confirm-apple-roi','--near-apple-geometry','--near-apple-gap-m','.15']
            write_json(out/'observer-startup.json',dict(command=command,robot_commands_sent=False))
            observer_log=(out/'observe.log').open('x');observer=subprocess.Popen(command,stdout=observer_log,stderr=subprocess.STDOUT,start_new_session=True)
            seen=set();end=time.monotonic()+105;reason='waiting_for_new_three_frame_identity'
            while len(rows)<5:
                if observer.poll() is not None:raise RuntimeError('new_observer_exited')
                if time.monotonic()>end:raise RuntimeError('new_close_geometry_wait_expired:'+reason)
                try:
                    path=latest(recording);snapshot=json.loads((path.parent/'snapshot.json').read_text())
                    transform=snapshot['tracking_transforms']['odom_from_base_at_depth'];rot=rotation(transform['quaternion'])
                    odom=[*transform['translation'][:2],math.atan2(rot[1,0],rot[0,0])]
                    evidence=read_latest(recording,closure=True,now_ns=time.time_ns(),epoch=epoch,task_id=task,
                        current_odom=odom,palm_height=.2771978151685961+ready['joints']['arm_lift_joint'],expected_room='bedroom')
                    actual=Path(evidence['evidence']).parent;snapshot=json.loads((actual/'snapshot.json').read_text())
                    fresh_snapshot(snapshot,time.time_ns(),epoch)
                    if snapshot['streams']['depth']['stamp_ns']!=evidence['stamp_ns']:raise ValueError('current_frame_binding_required')
                    if str(actual) in seen:time.sleep(.01);continue
                    pose=map_pose(snapshot)
                    if math.dist(pose[:2],ready['base_map_pose'][:2])>.01:raise RuntimeError('robot_moved_during_readonly')
                    row=dict(capture=str(actual),mode=evidence['observation_mode'],stamp_ns=evidence['stamp_ns'],
                        target_base_m=evidence['target_base_m'],error_xyz_m=evidence['center_minus_contact_xyz_m'],
                        checked_ns=time.time_ns(),map_pose=pose)
                    rows.append(row);seen.add(str(actual));write_json(out/'readonly-samples.json',rows)
                    print(json.dumps(dict(sample=len(rows),mode=row['mode'],target=row['target_base_m'])),flush=True)
                except (ValueError,KeyError,FileNotFoundError) as exc:reason=str(exc)
                time.sleep(.01)
            if not any(row['mode']=='near_geometry_tracked' for row in rows):raise RuntimeError('fresh_no_yolo_handoff_not_observed')
            save('corrected_r14_grasp_runner',new_readonly_close_samples=len(rows))
            command=[sys.executable,str(code/'run_bedroom_apple_regrasp.py'),'--run','--output',str(out/'grasp'),
                '--model-python',cfg['model_python'],'--registry',cfg['registry'],'--environment',cfg['environment'],
                '--observer-recording',str(recording),'--observer-epoch',epoch,'--task-id',task,
                '--contact-start-lift',str(ready['joints']['arm_lift_joint']),
                '--verify-preclose-rgbd-follow','--motion-profile',cfg.get('motion_profile','baseline')]
            write_json(out/'grasp-intent.json',dict(command=command,prior_current_readonly_samples=rows,
                old_approach_replayed=False,requires_independent_child_publish_time_checks=True))
            with (out/'grasp.log').open('x') as log:
                child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                exit_code=child.wait();child=None
            result=json.loads((out/'grasp/result.json').read_text());report['grasp_result']=result
            report['base_approach_commands_sent']=len(result.get('approach',[]))
            passed=exit_code==0 and result.get('grasp_success_verified') is True and not result.get('approach')
            save('new_close_lift_verified' if passed else 'new_close_lift_not_verified',runner_exit=exit_code,
                base_approach_commands_sent=report['base_approach_commands_sent'],grasp_success_verified=passed,
                reason=None if passed else result.get('reason',result.get('status')))
        except (Exception,KeyboardInterrupt) as exc:save('stopped',reason=str(exc) or 'interrupted',requires_actual_pose_audit=True)
        finally:
            if child is not None:stop_group(child)
            if observer is not None:stop_group(observer)
            if observer_log is not None:observer_log.close()
            write_json(out/'result.json',dict(report,readonly_samples=rows))
        raise SystemExit(0 if report.get('status')=='new_close_lift_verified' else 1)


if __name__=='__main__':main()
