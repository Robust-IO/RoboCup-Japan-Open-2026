"""One explicitly reviewed finite Unity carry adjustment; never opens the hand.

Simulation collision override is explicit and recorded, not propagated to Nav2.
Finite receiver trajectories cannot be cancelled by publishing Twist zero.
"""
import argparse
import json
import math
import os
import time
from pathlib import Path
from motion_speed import duration_fields, finite_duration, load_profile
from task_outcome_policy import pose_matches


def adjustment_settled(error_m, yaw_error_rad, spread_m, *, single_placement=False):
    if not all(math.isfinite(v) and v>=0 for v in (error_m,yaw_error_rad,spread_m)):
        return False
    # The next fresh measured support check decides placement, not odometry
    # matching a nominal endpoint within8mm. Other finite moves keep their fence.
    return spread_m<=.02 if single_placement else error_m<=.008 and yaw_error_rad<=.015 and spread_m<=.005


def endpoint(start, delta, *, max_translation_m=.4):
    if max_translation_m not in (.4,.6,None):raise ValueError('unsupported_translation_limit')
    if len(start)!=3 or len(delta)!=3 or not all(math.isfinite(v) for v in (*start,*delta)):
        raise ValueError('invalid_pose')
    distance=math.hypot(*delta[:2])
    if not math.isfinite(distance) or (max_translation_m is not None and distance>max_translation_m) or abs(delta[2])>.35:
        raise ValueError('outside_finite_adjustment_range')
    x,y,a=start;dx,dy,da=delta
    value=[x+math.cos(a)*dx-math.sin(a)*dy,y+math.sin(a)*dx+math.cos(a)*dy,a+da]
    if not all(math.isfinite(v) for v in value):raise ValueError('nonfinite_endpoint')
    return value


def single_placement_limit(delta,arm,opening,identity_recording=None):
    """Explicit one-shot simulated placement, not a general motion-limit change."""
    endpoint([0,0,0],delta,max_translation_m=None)
    if (identity_recording or delta[0]<=0 or abs(delta[2])>1e-6
            or len(arm)!=5 or not all(math.isfinite(x) for x in (*arm,opening))
            or not .02<opening<.62 or not .07<=arm[0]<=.443
            or any(abs(x-y)>.003 for x,y in zip(arm[1:],[-1.7,0.,.1292036732051034,0.]))):
        raise ValueError('single_placement_requires_forward_horizontal_holding_pose')
    return None  # Explicit single-placement path only: no translation cap.


def single_grasp_limit(delta,arm,opening,identity_recording):
    endpoint([0,0,0],delta,max_translation_m=None)
    if (not identity_recording or delta[0]<=0 or abs(delta[2])>1e-6
            or len(arm)!=5 or not all(math.isfinite(x) for x in (*arm,opening))
            or abs(opening-.65)>.001 or not .24<=arm[0]<=.30
            or any(abs(x-y)>.003 for x,y in zip(arm[1:],[-1.7,0.,.1292036732051034,0.]))):
        raise ValueError('single_grasp_requires_forward_horizontal_empty_identity_pose')
    return None  # Final fresh lease still checks target height/direction/overshoot.


def terminal_grasp_limit(delta,arm,opening,identity_recording):
    endpoint([0,0,0],delta)
    if (not identity_recording or math.hypot(*delta[:2])<=0 or abs(delta[2])>1e-6
            or len(arm)!=5 or not all(math.isfinite(x) for x in (*arm,opening))
            or abs(opening-.65)>.001 or not .24<=arm[0]<=.30
            or any(abs(x-y)>.003 for x,y in zip(arm[1:],[-1.7,0.,.1292036732051034,0.]))):
        raise ValueError('terminal_correction_requires_horizontal_empty_identity_pose')
    return .4  # Existing finite controller envelope; fresh target region is tighter.


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--simulation-ignore-collision',action='store_true')
    p.add_argument('--reviewed-map-pose',nargs=3,type=float,required=True)
    p.add_argument('--delta',nargs=3,type=float,required=True)
    p.add_argument('--expected-arm',nargs=5,type=float,required=True)
    p.add_argument('--holding-opening',type=float,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--identity-recording',type=Path);p.add_argument('--tracking-epoch');p.add_argument('--tracking-task-id');p.add_argument('--expected-room',choices=['living_room','bedroom'],default='living_room')
    p.add_argument('--motion-profile',choices=['baseline','balanced_v1'],default='baseline')
    p.add_argument('--single-placement-approach',action='store_true',help='Explicit forward horizontal held placement in ONE finite command, no distance cap; never split or retry')
    p.add_argument('--single-grasp-approach',action='store_true')
    p.add_argument('--grasp-terminal-correction',action='store_true')
    p.add_argument('--correction-after',type=Path)
    p.add_argument('--efficient-transfer',action='store_true')
    p.add_argument('--joint-feedback-max-age-s',type=float,choices=(.4,.6),default=.4)
    a=p.parse_args()
    if (a.grasp_terminal_correction and (not a.efficient_transfer or a.single_grasp_approach or a.single_placement_approach)
            or bool(a.correction_after)!=a.grasp_terminal_correction):
        raise ValueError('exclusive_measured_terminal_correction_required')
    if a.single_grasp_approach and (a.single_placement_approach or not a.efficient_transfer):
        raise ValueError('exclusive_efficient_single_grasp_required')
    translation_limit=single_placement_limit(a.delta,a.expected_arm,a.holding_opening,a.identity_recording) if a.single_placement_approach else .4
    if a.single_grasp_approach:
        translation_limit=single_grasp_limit(a.delta,a.expected_arm,a.holding_opening,a.identity_recording)
    previous=None
    if a.grasp_terminal_correction:
        from grasp_terminal_correction import completed_context
        translation_limit=terminal_grasp_limit(a.delta,a.expected_arm,a.holding_opening,a.identity_recording)
        if a.correction_after.resolve().parent!=a.output.resolve().parent:
            raise ValueError('terminal_predecessor_outside_current_task')
        previous=json.loads(a.correction_after.read_text());completed_context(previous)
    endpoint([0,0,0],a.delta,max_translation_m=translation_limit)
    speed_profile=load_profile(a.motion_profile)
    duration=finite_duration(speed_profile,a.delta,holding=.02<a.holding_opening<.62,max_translation_m=translation_limit)
    if not a.run or not a.simulation_ignore_collision or os.environ.get('ROS_DOMAIN_ID')!='71':raise ValueError('explicit_simulation_override_required')
    if a.output.exists() or not all(math.isfinite(v) for v in (*a.reviewed_map_pose,*a.expected_arm,a.holding_opening)) or not .02<=a.holding_opening<=.65:raise ValueError('invalid_reviewed_pose_or_output')
    if a.identity_recording and (not a.tracking_epoch or not a.tracking_task_id or abs(a.delta[2])>.000001 or not a.single_grasp_approach and math.hypot(*a.delta[:2])>.25 or abs(a.holding_opening-.65)>.001):raise ValueError('invalid_measured_empty_grasp_approach')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import JointState,LaserScan
    from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
    from tf2_ros import Buffer,TransformListener
    from fresh_feedback_wait import wait_for_fresh
    from transfer_efficiency import wait_ready
    rclpy.init();n=rclpy.create_node('reviewed_carry_adjustment');b=Buffer();listener=TransformListener(b,n);data={};pub=None
    names=['arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint']
    expected=a.expected_arm+[a.holding_opening]
    subs=[n.create_subscription(JointState,'/hsrb/joint_states',lambda m:data.update(joints=m),qos_profile_sensor_data),n.create_subscription(LaserScan,'/hsrb/base_scan',lambda m:data.update(scan=m),qos_profile_sensor_data)]
    topic='/hsrb/omni_base_controller/command'
    report=dict(command_sent=False,grip_commands_sent=False,simulation_ignore_collision=True,collision_certified=False,automatic_retry=False,motion_profile=a.motion_profile,trajectory_duration_s=duration)
    report.update(single_placement_approach=a.single_placement_approach,single_grasp_approach=a.single_grasp_approach,
                  grasp_terminal_correction=a.grasp_terminal_correction,
                  efficient_non_navigation=a.efficient_transfer,max_translation_m=translation_limit)
    if a.identity_recording:
        report['identity_context']=dict(recording=str(a.identity_recording.resolve()),epoch=a.tracking_epoch,task_id=a.tracking_task_id)
    if a.grasp_terminal_correction:report['correction_after']=str(a.correction_after.resolve())
    def pump(timeout=.01):
        rclpy.spin_once(n,timeout_sec=timeout)
        for _ in range(30):rclpy.spin_once(n,timeout_sec=0.)
    def age(stamp):return (n.get_clock().now()-Time.from_msg(stamp)).nanoseconds/1e9
    def pose(frame):
        tf=b.lookup_transform(frame,'base_footprint',Time())
        if not -.02<=age(tf.header.stamp)<=.4:raise RuntimeError('stale_base_tf')
        t=tf.transform.translation;q=tf.transform.rotation
        return [t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    def check():
        if len(data)!=2:raise RuntimeError('missing_feedback')
        if any(not -.02<=age(m.header.stamp)<=(a.joint_feedback_max_age_s if key=='joints' else .4)
                for key,m in data.items()):raise RuntimeError('stale_capture')
        j=data['joints'];values=dict(zip(j.name,j.position))
        if any(k not in values for k in names) or not pose_matches([values[k] for k in names],expected):raise RuntimeError('holding_pose_changed')
        for t in ('/hsrb/command_velocity','/hsrb/arm_trajectory_controller/command','/hsrb/gripper_controller/command','/hsrb/head_trajectory_controller/command'):
            if n.count_publishers(t):raise RuntimeError('other_motion_controller_present:'+t)
        if n.count_publishers(topic)!=(1 if pub else 0) or n.count_subscribers(topic)!=1:raise RuntimeError('base_control_not_exclusive')
        return pose('odom')
    def fresh():return wait_for_fresh(check,pump,lambda:None)
    try:
        if a.efficient_transfer:
            def ready():
                if len(data)!=2 or not all(b.can_transform(f,'base_footprint',Time()) for f in ('map','odom')):
                    return False
                if n.count_subscribers(topic)!=1:return False
                try:check()
                except RuntimeError as exc:
                    if str(exc) in ('stale_capture','stale_base_tf'):return False
                    raise
                return True
            wait_ready(ready,pump)
        else:
            end=time.monotonic()+4
            while time.monotonic()<end:pump(.03)
        start=fresh();mapped=pose('map');report.update(start_odom=start,start_map=mapped,delta=a.delta)
        if math.dist(mapped[:2],a.reviewed_map_pose[:2])>.02 or abs(math.atan2(math.sin(mapped[2]-a.reviewed_map_pose[2]),math.cos(mapped[2]-a.reviewed_map_pose[2])))>.02:raise RuntimeError('reviewed_start_mismatch')
        target=endpoint(start,a.delta,max_translation_m=translation_limit);report['target_odom']=target
        valid=[v for v in data['scan'].ranges if math.isfinite(v) and v>data['scan'].range_min]
        report['laser_min_warning_only_m']=min(valid) if valid else None
        pub=n.create_publisher(JointTrajectory,topic,10)
        if a.efficient_transfer:
            wait_ready(lambda:pub.get_subscription_count()==1 and n.count_publishers(topic)==1,
                       pump)
        else:
            end=time.monotonic()+.5
            while time.monotonic()<end:pump(.02)
        if math.dist(fresh()[:2],start[:2])>.005:raise RuntimeError('base_moved_before_send')
        if a.identity_recording:
            from apple_approach_lease import read_latest
            from inverse_table_place_plan import OFFSET
            end=time.monotonic()+3
            while True:
                current=fresh()
                try:
                    if a.grasp_terminal_correction:
                        kwargs=dict(terminal_correction=True,previous=previous,recording=a.identity_recording)
                    else:
                        kwargs=dict(max_step_m=None,single_approach=True) if a.single_grasp_approach else dict(max_step_m=.25)
                        kwargs['min_forward_gap_m']=.01
                    report['fresh_identity_evidence']=read_latest(a.identity_recording,now_ns=n.get_clock().now().nanoseconds,epoch=a.tracking_epoch,task_id=a.tracking_task_id,current_odom=current,translation=a.delta[:2],palm_height=OFFSET+a.expected_arm[0],expected_room=a.expected_room,**kwargs)
                    break
                except (ValueError,FileNotFoundError) as exc:
                    retryable=isinstance(exc,(FileNotFoundError,json.JSONDecodeError)) or str(exc) in ('vision_evidence_expired_or_unsynchronised','current_identity_not_confirmed','vision_result_not_fresh','no_current_identity_record')
                    if not retryable or time.monotonic()>=end:raise
                    pump(.01)
        msg=JointTrajectory();msg.joint_names=['odom_x','odom_y','odom_t'];pt=JointTrajectoryPoint();pt.positions=target
        pt.time_from_start.sec,pt.time_from_start.nanosec=duration_fields(duration);msg.points=[pt]
        pub.publish(msg);report['command_sent']=True;print(json.dumps(dict(event='finite_adjustment_sent',target_odom=target)),flush=True)
        sent=time.monotonic();history=[];faults=[]
        # Observe through the entire bounded command, including on transient loss.
        while time.monotonic()-sent<duration+2:
            pump(.02)
            try:history.append(dict(t=time.monotonic()-sent,pose=check()))
            except Exception as exc:faults.append(str(exc))
        recent=[x['pose'] for x in history if x['t']>=duration+1]
        if not recent:raise RuntimeError('no_final_feedback')
        final=fresh();err=math.dist(final[:2],target[:2]);yawerr=abs(math.atan2(math.sin(final[2]-target[2]),math.cos(final[2]-target[2])))
        spread=max(math.dist(x[:2],final[:2]) for x in recent)
        report.update(final_odom=final,final_map=pose('map'),error_m=err,yaw_error_rad=yawerr,spread_m=spread,monitor_faults=sorted(set(faults)))
        report.update(planned_endpoint_is_success_gate=not a.single_placement_approach,
                      actual_release_region_must_be_checked_separately=a.single_placement_approach)
        if not adjustment_settled(err,yawerr,spread,single_placement=a.single_placement_approach):raise RuntimeError('endpoint_not_verified')
        if any(f not in ('stale_capture','stale_base_tf') and 'extrapolation' not in f.lower() for f in faults):raise RuntimeError('nontransient_monitor_fault')
        report['status']='endpoint_verified'
        report['motion_completed_at_ns']=n.get_clock().now().nanoseconds
    except Exception as exc:report.update(status='failed',reason=str(exc))
    finally:
        a.output.write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True);n.destroy_node();rclpy.shutdown()
    if report['status']!='endpoint_verified':raise SystemExit(1)


if __name__=='__main__':main()
