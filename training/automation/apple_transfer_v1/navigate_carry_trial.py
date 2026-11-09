"""Single explicit simulated carry navigation; never releases grip or reports task completion."""
import argparse,json,math,time,os
import signal
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from sensor_msgs.msg import JointState,LaserScan
from tf2_ros import Buffer,TransformListener
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters, SetParametersAtomically
from lifecycle_msgs.srv import GetState
from navigation_feedback_gate import validate_ages, acquire_fresh_feedback, NavigationFeedbackError
from navigation_feedback_recovery import recover_stopped_feedback
from motion_speed import NAV_SPEED_NAMES, NavigationSpeedLease, load_profile, navigation_caps


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--goal',nargs=3,type=float,required=True);p.add_argument('--waypoint',nargs=3,type=float,action='append',default=[]);p.add_argument('--holding-opening',type=float,default=.4122018066);p.add_argument('--precise',action='store_true');p.add_argument('--verify-current-precision',action='store_true',help='Read/check .06/.08/.06 only; never set goal tolerances');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--motion-profile',choices=['baseline','balanced_v1','carry_fast_v1'],default='baseline')
    p.add_argument('--joint-feedback-max-age-s',type=float,choices=(.7,.85),default=.7)
    p.add_argument('--recover-feedback-interruption',action='store_true');a=p.parse_args()
    speed_profile=load_profile(a.motion_profile)
    if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71':raise ValueError('Explicit simulation run required')
    if not all(math.isfinite(v) for pose in a.waypoint+[a.goal] for v in pose) or a.output.exists() or not 0<=a.holding_opening<=.65 or a.precise and a.verify_current_precision:raise ValueError('Invalid goal/output/grip/precision mode')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=rclpy.create_node('phase6_carry_trial');b=Buffer();listener=TransformListener(b,n);data={};h=None;report=dict(target=a.goal,grip_commands_sent=False,goal_precision_changed=False,parameters_changed=False,motion_profile=a.motion_profile,speed_contexts=[])
    report['joint_feedback_max_age_s']=a.joint_feedback_max_age_s
    report['feedback_recovery_enabled']=a.recover_feedback_interruption
    report['feedback_recoveries']=0
    subs=[n.create_subscription(JointState,'/hsrb/joint_states',lambda m:data.update(joints=m),qos_profile_sensor_data),n.create_subscription(LaserScan,'/hsrb/base_scan',lambda m:data.update(scan=m),qos_profile_sensor_data)]
    client=ActionClient(n,NavigateToPose,'/navigate_to_pose');manager=n.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
    state_client=n.create_client(GetState,'/controller_server/get_state')
    setter=n.create_client(SetParameters,'/controller_server/set_parameters');getter=n.create_client(GetParameters,'/controller_server/get_parameters');original=None
    speed_setter=n.create_client(SetParametersAtomically,'/controller_server/set_parameters_atomically')
    parameter_names=['general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','FollowPath.xy_goal_tolerance']
    def set_params(values):
        if not setter.wait_for_service(timeout_sec=5):raise RuntimeError('parameter_service_unavailable')
        req=SetParameters.Request();req.parameters=[Parameter(k,value=float(v)).to_parameter_msg() for k,v in zip(parameter_names,values)]
        result=wait(setter.call_async(req),5)
        if not all(r.successful for r in result.results):raise RuntimeError('precision_parameters_rejected:'+str(result.results))
    def wait(f,timeout):
        end=time.monotonic()+timeout
        while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
        if not f.done():raise RuntimeError('service_timeout')
        return f.result()
    def read_speeds():
        if not getter.wait_for_service(timeout_sec=5):raise RuntimeError('speed_parameter_service_unavailable')
        request=GetParameters.Request();request.names=list(NAV_SPEED_NAMES)
        return [v.double_value for v in wait(getter.call_async(request),5).values]
    def write_speeds(values):
        if not speed_setter.wait_for_service(timeout_sec=5):raise RuntimeError('atomic_speed_parameter_service_unavailable')
        request=SetParametersAtomically.Request()
        request.parameters=[Parameter(k,value=float(v)).to_parameter_msg() for k,v in zip(NAV_SPEED_NAMES,values)]
        result=wait(speed_setter.call_async(request),5).result
        if not result.successful:raise RuntimeError('navigation_speed_parameters_rejected:'+result.reason)
    speed_lease=NavigationSpeedLease(read_speeds,write_speeds)
    def lifecycle(command):
        if not manager.wait_for_service(timeout_sec=5):raise RuntimeError('manager_unavailable')
        req=ManageLifecycleNodes.Request();req.command=command
        if not wait(manager.call_async(req),30).success:raise RuntimeError('lifecycle_failed')
    def reset_if_configured():
        if not state_client.wait_for_service(timeout_sec=5):raise RuntimeError('controller_state_unavailable')
        state=wait(state_client.call_async(GetState.Request()),5).current_state.id
        if state!=1:lifecycle(3)
    def check_once():
        # Services/action feedback and TF share this executor. Consume ready
        # callbacks before inspecting the newest received samples; no stale
        # sample is accepted. The explicit joint-age option applies to the
        # fixed carry posture; scan/TF and all goal tolerances keep their limits.
        for _ in range(20):rclpy.spin_once(n,timeout_sec=0)
        if len(data)!=2:raise RuntimeError('missing_feedback')
        ages={key:(n.get_clock().now()-Time.from_msg(m.header.stamp)).nanoseconds/1e9 for key,m in data.items()}
        j=dict(zip(data['joints'].name,data['joints'].position))
        for name,value in {'arm_lift_joint':0.,'arm_flex_joint':0.,'arm_roll_joint':0.,'wrist_flex_joint':-math.pi/2,'wrist_roll_joint':0.,'hand_motor_joint':a.holding_opening}.items():
            if name not in j or not math.isfinite(j[name]) or abs(j[name]-value)>.02:raise RuntimeError('carry_pose_changed:'+name)
        tf=b.lookup_transform('map','base_footprint',Time())
        ages['tf']=(n.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
        try:validate_ages(ages,joint_max_age_s=a.joint_feedback_max_age_s)
        except RuntimeError:
            report['feedback_fault']=dict(ages_s=ages,checked_ns=n.get_clock().now().nanoseconds,
                stamps_ns={key:Time.from_msg(m.header.stamp).nanoseconds for key,m in data.items()})
            raise
        t=tf.transform.translation;q=tf.transform.rotation
        pose=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        report['last_verified_feedback']=dict(pose=pose,ages_s=ages,checked_ns=n.get_clock().now().nanoseconds,
            stamps_ns=dict(scan=Time.from_msg(data['scan'].header.stamp).nanoseconds,tf=Time.from_msg(tf.header.stamp).nanoseconds))
        return pose
    def feedback_event(event):
        report.setdefault('feedback_acquisitions',[]).append(dict(checked_ns=n.get_clock().now().nanoseconds,**event))
    def check():
        # Only re-read/pump NEW messages. The same Nav2 goal remains active;
        # cancellation can be deferred by at most150ms, never by a new budget
        # for each stream. No old pose/sample is returned on expiry.
        return acquire_fresh_feedback(check_once,lambda dt:rclpy.spin_once(n,timeout_sec=dt),on_event=feedback_event,
            joint_max_age_s=a.joint_feedback_max_age_s)
    try:
        end=time.monotonic()+4
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
        report['start']=check()
        if n.count_publishers('/hsrb/omni_base_controller/command'):raise RuntimeError('finite_base_controller_present')
        reset_if_configured();lifecycle(0)
        if a.verify_current_precision:
            if not getter.wait_for_service(timeout_sec=5):raise RuntimeError('parameter_service_unavailable')
            req=GetParameters.Request();req.names=parameter_names
            current=[v.double_value for v in wait(getter.call_async(req),5).values]
            if any(abs(x-y)>1e-8 for x,y in zip(current,[.06,.08,.06])):raise RuntimeError('current_precision_differs_from_verified_trial')
            report.update(current_precision=current,goal_precision_changed=False)
        if a.precise:
            if not getter.wait_for_service(timeout_sec=5):raise RuntimeError('parameter_service_unavailable')
            req=GetParameters.Request();req.names=parameter_names
            original=[v.double_value for v in wait(getter.call_async(req),5).values]
            report['goal_precision_changed']=True
            set_params([.06,.08,.06]);report['temporary_precision']=[.06,.08,.06]
            # DWB critics cache xy_goal_tolerance when configured. Merely updating
            # the parameter at runtime lets RotateToGoal stop at the old .20m.
            # Reconfigure without any active goal so checker and critics agree.
            lifecycle(3);lifecycle(0)
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('navigation_unavailable')
        report['reached']=[]
        for index,target in enumerate(a.waypoint+[a.goal]):
            before=check();distance=math.dist(before[:2],target[:2])
            caps=navigation_caps(speed_profile,distance,holding=.02<a.holding_opening<.62)
            speed_lease.apply(caps)
            report['speed_contexts'].append(dict(waypoint=index,start_distance_m=distance,requested_caps=caps))
            while True:
                goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg()
                goal.pose.pose.position.x=target[0];goal.pose.pose.position.y=target[1];goal.pose.pose.orientation.z=math.sin(target[2]/2);goal.pose.pose.orientation.w=math.cos(target[2]/2)
                # Recheck immediately before dispatch; never send on recovered old data.
                check()
                h=wait(client.send_goal_async(goal),5)
                if not h.accepted:raise RuntimeError('goal_rejected')
                print('Carry goal accepted '+str(index)+' '+str(target),flush=True);f=h.get_result_async();last=time.monotonic()
                try:
                    while not f.done():
                        rclpy.spin_once(n,timeout_sec=.05);pose=check()
                        if time.monotonic()-last>5:print(json.dumps(dict(waypoint=index,pose=pose)),flush=True);last=time.monotonic()
                    result=f.result();pose=check()
                    break
                except NavigationFeedbackError as exc:
                    if (not a.recover_feedback_interruption or report['feedback_recoveries']>=5
                            or type(exc.age_s) not in (int,float) or not math.isfinite(exc.age_s) or exc.age_s<0):raise
                    def cancel_current():
                        if not f.done():
                            wait(h.cancel_goal_async(),5)
                            terminal=wait(f,5)
                        else:terminal=f.result()
                        if terminal.status not in (4,5):raise RuntimeError('navigation_cancel_not_confirmed')
                    def fresh_stopped():
                        check_once();return report['last_verified_feedback']
                    report['feedback_recoveries']+=1
                    feedback_event(dict(event='stopping_unfinished_waypoint_for_feedback',waypoint=index,reason=str(exc)))
                    recover_stopped_feedback(fresh_stopped,lambda dt:rclpy.spin_once(n,timeout_sec=dt),cancel_current,on_event=feedback_event)
                    if n.count_publishers('/hsrb/omni_base_controller/command'):raise RuntimeError('finite_base_controller_present')
                    # If Nav2 completed while cancelling, do not resend that goal.
                    if f.result().status==4:
                        result=f.result();pose=check();break
            error=math.dist(pose[:2],target[:2]);yaw_error=abs(math.atan2(math.sin(pose[2]-target[2]),math.cos(pose[2]-target[2])))
            report.update(action_status=result.status,final_pose=pose,position_error_m=error)
            report['reached'].append(dict(index=index,target=target,actual=pose,error_m=error,yaw_error_rad=yaw_error,status=result.status))
            report['last_completed_waypoint_pose']=pose
            print(json.dumps(report['reached'][-1]),flush=True)
            if result.status!=4:raise RuntimeError('navigation_failed')
            if (a.precise or a.verify_current_precision) and (error>.085 or yaw_error>.11):raise RuntimeError('actual_waypoint_precision_failed')
        report['status']='arrived'
    except (Exception,KeyboardInterrupt) as e:
        report.update(status='failed',error=str(e) or 'interrupted')
        # A previous successful waypoint is NOT the physical stop pose.
        report.pop('final_pose',None);report.pop('position_error_m',None)
        if h is not None and h.accepted:
            try:wait(h.cancel_goal_async(),5)
            except Exception as exc:report['cancel_error']=str(exc)
    finally:
        if original is not None:
            try:set_params(original);report['precision_restored']=original
            except Exception as exc:report['parameter_restore_error']=str(exc)
        try:reset_if_configured();report['nav_reset']=True
        except Exception as exc:report['nav_reset_error']=str(exc);report['status']='failed'
        try:
            speed_lease.restore();report['speed_parameters_restored']=True
        except Exception as exc:
            report.update(speed_parameters_restored=False,speed_restore_error=str(exc),status='failed')
        if report.get('status')!='arrived':
            try:
                report['post_stop_pose']=check()
                report['post_stop_pose_is_fresh_at_check']=True
            except Exception as exc:
                report['post_stop_pose_is_fresh_at_check']=False
                report['post_stop_pose_error']=str(exc)
        report.update(speed_parameters_changed=speed_lease.changed,parameters_changed=bool(original is not None or speed_lease.changed))
        a.output.write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True);n.destroy_node();rclpy.shutdown()
    if report['status']!='arrived':raise SystemExit(1)


if __name__=='__main__':main()
