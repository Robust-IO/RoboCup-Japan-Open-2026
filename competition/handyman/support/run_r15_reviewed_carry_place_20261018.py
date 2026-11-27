"""ONE new continuation from verified held apple; never replay grasp/old journal."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

import numpy as np
from PIL import Image
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');base=Path('/__HANDYMAN_PACKAGE__/runtime')
sys.path.insert(0,str(code))
from apple_transfer_auto_backend import LiveBackend,read
from apple_transfer_auto_plan import map_pose
from apple_transfer_automation import RobotLock,check_inputs,write_json
from audit_preclose_identity_lift_follow import audit
from hand_camera_ray import rotation
from tracked_apple_surface import measure
from verify_apple_lift_sequence import palm_in_base


def palm_rotation(snapshot):
    return rotation(snapshot['base_from_hand']['quaternion'])@rotation(snapshot['palm_from_hand']['quaternion']).T


def held_audit(backend,capture,offset_palm,reference_radius,expected_arm):
    rows=[]
    for path in sorted(capture.parent.glob('*/snapshot.json')):
        snapshot=read(path);joints=snapshot['joints']
        names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        if any(abs(joints[key]-value)>.006 for key,value in zip(names,[*expected_arm,backend.holding])):
            raise ValueError('current_holding_pose_changed')
        expected=palm_in_base(snapshot)+palm_rotation(snapshot)@offset_palm
        geometry=measure(np.asarray(Image.open(path.parent/'rgb.png').convert('RGB')),
            np.load(path.parent/'depth_m.npy',allow_pickle=False),snapshot,expected)
        if abs(geometry['radius_m']-reference_radius)>.008:raise ValueError('known_held_apple_size_changed')
        actual=palm_rotation(snapshot).T@(np.asarray(geometry['center_base'])-palm_in_base(snapshot))
        rows.append(dict(capture=str(path.parent),stamp_ns=snapshot['streams']['depth']['stamp_ns'],
            measurement=geometry,palm_relative_offset_m=actual.tolist(),base_map_pose=map_pose(snapshot)))
    span=(rows[-1]['stamp_ns']-rows[0]['stamp_ns'])/1e9 if len(rows)>1 else 0
    spread=max((float(np.linalg.norm(np.asarray(row['palm_relative_offset_m'])-rows[-1]['palm_relative_offset_m'])) for row in rows),default=1.)
    if len(rows)<3 or span<.4 or spread>.01:raise ValueError('current_multiframe_held_geometry_unverified')
    return dict(held_object_verified=True,frames=len(rows),span_s=span,palm_relative_center_spread_m=spread,
        rows=rows,robot_commands_sent=False,basis='Original actual grasp identity plus NEW RGBD measured surface; current palm rotation accounted for')


def check_control(output,reference):
    """Read only; action children independently check fresh ownership again."""
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer,TransformListener
    from rcl_interfaces.srv import GetParameters
    from lifecycle_msgs.srv import GetState
    from fresh_feedback_wait import wait_for_fresh
    from inspect_transfer_ready import validate_sample_times
    rclpy.init();node=rclpy.create_node('r15_holding_control_readonly');buf=Buffer();listener=TransformListener(buf,node)
    received={};subscription=node.create_subscription(JointState,'/hsrb/joint_states',lambda m:received.update(joints=m),qos_profile_sensor_data)
    report=dict(ready=False,robot_commands_sent=False)
    def call(client,request):
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('read_service_missing')
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done():raise RuntimeError('read_service_timeout')
        return future.result()
    def current():
        for _ in range(20):rclpy.spin_once(node,timeout_sec=0)
        if 'joints' not in received:raise RuntimeError('stale_joints')
        message=received['joints'];mapped=buf.lookup_transform('map','base_footprint',Time());odom=buf.lookup_transform('odom','base_footprint',Time())
        now=node.get_clock().now().nanoseconds
        report['feedback_ages_s']=validate_sample_times(Time.from_msg(message.header.stamp).nanoseconds,Time.from_msg(mapped.header.stamp).nanoseconds,now)
        if not -.02<=(now-Time.from_msg(odom.header.stamp).nanoseconds)/1e9<=.4:raise RuntimeError('stale_base_tf')
        return message,mapped,odom
    def pose(tf):
        t=tf.transform.translation;q=tf.transform.rotation
        return [t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    try:
        end=time.monotonic()+4
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        request=GetParameters.Request();request.names=['general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','FollowPath.xy_goal_tolerance']
        tolerances=[value.double_value for value in call(node.create_client(GetParameters,'/controller_server/get_parameters'),request).values]
        state=call(node.create_client(GetState,'/controller_server/get_state'),GetState.Request()).current_state.id
        message,mapped,odom=wait_for_fresh(current,lambda dt:rclpy.spin_once(node,timeout_sec=dt))
        joints=dict(zip(message.name,message.position));actual=pose(mapped);actual_odom=pose(odom)
        topics=['/hsrb/'+tail for tail in ('command_velocity','omni_base_controller/command','arm_trajectory_controller/command','gripper_controller/command','head_trajectory_controller/command')]
        publishers={topic:node.count_publishers(topic) for topic in topics};subscribers={topic:node.count_subscribers(topic) for topic in topics[1:]}
        report.update(joints=joints,base_map_pose=actual,base_odom_pose=actual_odom,navigation_tolerances=tolerances,
            controller_state=state,command_publishers=publishers,command_subscribers=subscribers,checked_ns=node.get_clock().now().nanoseconds)
        if any(publishers.values()) or any(n!=1 for n in subscribers.values()) or state not in (1,2):raise ValueError('current_control_not_exclusive_or_idle')
        if len(tolerances)!=3 or any(abs(a-b)>1e-8 for a,b in zip(tolerances,[.06,.08,.06])):raise ValueError('navigation_parameters_changed_no_writes')
        for key in ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint','head_pan_joint','head_tilt_joint'):
            if key not in joints or not math.isfinite(joints[key]) or abs(joints[key]-reference['joints'][key])>.006:raise ValueError('holding_posture_changed:'+key)
        for value,key in ((actual,'map_from_base_at_depth'),(actual_odom,'odom_from_base_at_depth')):
            old=reference['tracking_transforms'][key];r=rotation(old['quaternion']);yaw=math.atan2(r[1,0],r[0,0])
            if math.dist(value[:2],old['translation'][:2])>.02 or abs(math.atan2(math.sin(value[2]-yaw),math.cos(value[2]-yaw)))>.02:
                raise ValueError('scene_or_robot_changed_since_holding_checkpoint')
        report['ready']=True
    except Exception as exc:report['reason']=str(exc)
    finally:
        write_json(output,report);node.destroy_node();rclpy.try_shutdown()
    if report['ready'] is not True:raise ValueError('holding_readiness:'+report.get('reason','not_ready'))
    return report


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run',action='store_true');args=parser.parse_args()
    if not args.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('explicit_domain71_simulation_required')
    out=base/'apple-r15-reviewed-carry-place-20261018-01';task='apple-r15-carry-place-'+uuid.uuid4().hex
    with RobotLock():
        cfg=read(code/'config/apple_transfer_auto_v1.json');inputs=check_inputs(cfg)
        checkpoint=read(base/'apple-close-lift-live-progress-20261018.json')
        source=Path(checkpoint['actual_trial']['directory']);trial_dir=source/'grasp/close-lift';trial=read(trial_dir/'result.json')
        if checkpoint['grasp_verified'] is not True or checkpoint['current_robot']['holding_object'] is not True:raise ValueError('verified_held_apple_checkpoint_required')
        prior=read(source/'input-files.json')
        if any(inputs[key]!=prior[key] for key in ('environment_sha256','semantic_map_sha256','model_sha256')):raise ValueError('assets_changed_since_actual_grasp')
        for name,digest in checkpoint['deployment']['hashes'].items():
            if hashlib.sha256((code/name).read_bytes()).hexdigest()!=digest:raise ValueError('r15_code_changed:'+name)
        if (cfg['policy']['placement_approach_mode']!='single' or cfg['policy']['max_placement_step_m'] is not None
            or cfg['policy']['edge_margin_m']!=.03 or cfg['policy']['edge_uncertainty_m']!=.01):raise ValueError('requested_placement_policy_changed')
        anchor=Path('/proc')/str(checkpoint['current_map_anchor_pid_last_observed'])/'cmdline'
        if 'publish_r13c_reviewed_map_anchor_20261018.py' not in anchor.read_bytes().replace(b'\0',b' ').decode():raise ValueError('current_map_anchor_changed_requires_review')
        out.mkdir(exist_ok=False)
        report=dict(status='running',run_kind='NEW_reviewed_verified_grasp_carry_and_placement',task_id=task,
            confirmed_stages=[],source_grasp=str(trial_dir),automatic_retry=False,automatic_resume=False,
            grasp_close_lift_or_source_approach_replayed=False,whole_uninterrupted_transfer_verified=False,
            task_finished_sent=False,does_not_exist_sent=False)
        with (out/'journal.jsonl').open('x') as stream:
            def record(event):
                stream.write(json.dumps(dict(time_ns=time.time_ns(),**event),allow_nan=False)+'\n');stream.flush();os.fsync(stream.fileno())
            b=LiveBackend(cfg,inputs);b.bind(out,task,record)
            write_json(out/'config.snapshot.json',cfg);b.evidence('input-files-audit',inputs)
            def step(name,operation):
                b.stage=name;report['stage']=name;write_json(out/'status.json',report)
                record(dict(event='intent',stage=name));print(json.dumps(dict(stage=name,status='starting')),flush=True)
                evidence=operation()
                if evidence.get('verified') is not True:raise ValueError('stage_not_verified:'+name)
                report['confirmed_stages'].append(name);record(dict(event='confirmed',stage=name,result=evidence));write_json(out/'status.json',report)
            def prepare_context():
                reference=read(Path(checkpoint['current_robot']['capture'])/'snapshot.json')
                control=check_control(out/'holding-readiness.json',reference)
                proof=audit(trial_dir,expected_recording=Path(trial['observation_directory']),
                    expected_epoch=trial['tracking_epoch'],expected_task_id=trial['tracking_task_id'])
                if proof['grasp_success_verified'] is not True:raise ValueError('corrected_actual_grasp_proof_not_verified')
                row=proof['rows'][-1];snapshot=read(Path(row['capture'])/'snapshot.json');candidate=row['center_measurement']
                center=np.asarray(candidate['center_m']);palm=palm_in_base(snapshot)
                b.offset_palm=palm_rotation(snapshot).T@(center-palm)
                before=read(Path(trial['close']['closure_identity_evidence']['evidence']).parent/'snapshot.json')
                b.grasp_plan=dict(contact_start_lift_m=before['joints']['arm_lift_joint'])
                b.holding=trial['close']['closure_evidence']['observed_opening_rad']
                b.held=dict(center_minus_palm_m=(center-palm).tolist(),bottom_below_palm_m=float(palm[2]-center[2]+candidate['radius_m']),
                    radius_m=candidate['radius_m'],holding_opening_rad=b.holding,verified_capture=row['capture'],new_identity_verified=False)
                b.initial_pose=control['base_map_pose']
                b.evidence('verified-held-geometry',b.held)
                return b.evidence('source-grasp-recomputed',proof)
            def audit_current(name,arm,initial=False):
                capture=b.capture(name);snapshot=read(capture/'snapshot.json');pose=map_pose(snapshot)
                value=held_audit(b,capture,b.offset_palm,b.held['radius_m'],arm)
                if initial and (b.room_at(pose)!='bedroom' or math.dist(pose[:2],b.initial_pose[:2])>.01):raise ValueError('robot_changed_before_carry')
                return b.evidence(name+'-audit',value)
            def retreat():
                capture=b.capture('before-source-retreat')
                evidence=b.adjust('source-clear-retreat',map_pose(read(capture/'snapshot.json')),[-.25,0.,0.],
                    [0.,0.,0.,-math.pi/2,0.],b.holding)
                return dict(verified=True,evidence=evidence)
            try:
                step('verify_recorded_grasp_and_current_control',prepare_context)
                step('verify_current_holding',lambda:audit_current('held-before-retract',
                    [b.grasp_plan['contact_start_lift_m']+.1,-1.7,0.,.1292036732051034,0.],True))
                step('retract_holding',b.retract_holding)
                step('verify_retracted_holding',lambda:audit_current('held-after-retract',[0.,0.,0.,-math.pi/2,0.]))
                step('source_clear_retreat',retreat)
                step('navigate_destination',b.navigate_destination)
                step('verify_destination_holding',lambda:audit_current('held-at-destination',[0.,0.,0.,-math.pi/2,0.]))
                for name in ('observe_surface','plan_placement','prepare_placement','approach_placement','check_release','release','withdraw','verify_placement'):
                    step(name,getattr(b,name))
                report.update(status='reviewed_r15_carry_placement_verified',placement_verified=True)
            except (Exception,KeyboardInterrupt) as exc:
                report.update(status='stopped',reason=str(exc) or 'interrupted',placement_verified=False,requires_new_physical_state_audit=True)
                record(dict(event='stopped',stage=report.get('stage'),reason=report['reason']))
            finally:
                b.close();write_json(out/'result.json',report);write_json(out/'status.json',report)
            print(json.dumps(report),flush=True)
            raise SystemExit(0 if report['status']=='reviewed_r15_carry_placement_verified' else 1)


if __name__=='__main__':main()
