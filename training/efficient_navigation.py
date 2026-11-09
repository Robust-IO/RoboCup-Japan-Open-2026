"""Simulation navigation with landmark preemption and persistent local recovery.

No arm/gripper commands. Intermediate poses are real measured pass points;
NavigateToPose preemption replaces the previous goal without a cancel/stop cycle.
The final furniture staging goal keeps the existing .06m/.08rad precision.
"""
import argparse,json,math,os,signal,time
from pathlib import Path
from navigation_efficiency_policy import (FeedbackPending,RetryState,build_landmarks,
    caps_for_pose,feedback_status,load_policy,pass_assessment,wrap)
from motion_speed import NavigationSpeedLease,NAV_SPEED_NAMES,load_profile


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true');p.add_argument('--goal',nargs=3,type=float,required=True)
    p.add_argument('--waypoint',nargs=3,type=float,action='append',default=[])
    p.add_argument('--holding-opening',type=float,default=.65)
    p.add_argument('--precise',action='store_true');p.add_argument('--verify-current-precision',action='store_true')
    p.add_argument('--motion-profile',choices=['efficiency_v1'],default='efficiency_v1')
    p.add_argument('--joint-feedback-max-age-s',type=float,choices=(.7,.85),default=.85)
    p.add_argument('--recover-feedback-interruption',action='store_true')
    p.add_argument('--semantic-map',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();policy=load_policy();profile=load_profile(a.motion_profile)
    if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('explicit_local_domain71_simulation_required')
    if a.output.exists() or not a.output.resolve().is_relative_to(Path('/mnt/d/handyman-training-runs')):raise ValueError('new_D_output_required')
    if not 0<=a.holding_opening<=.65 or not all(math.isfinite(x) for pose in [*a.waypoint,a.goal] for x in pose):raise ValueError('invalid_navigation_goal')
    if a.precise:raise ValueError('efficiency_navigation_preserves_existing_precision')
    doors=json.loads(a.semantic_map.read_text()).get('doorways',[])
    landmarks=build_landmarks(a.waypoint,a.goal,doors,policy)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.action import ActionClient
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState,LaserScan
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import ManageLifecycleNodes
    from rcl_interfaces.srv import GetParameters,SetParametersAtomically
    from lifecycle_msgs.srv import GetState
    from rclpy.parameter import Parameter
    from tf2_ros import Buffer,TransformListener
    from navigation_entry import repair_navigation,precision_matches,PRECISION_NAMES
    from gate_recovery import RecoveryStatus
    def interrupted(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('phase6_efficient_navigation');buffer=Buffer();listener=TransformListener(buffer,node)
    data={};started=time.monotonic();handles=[];last_caps=None;last_advisory=0.;tuning_before=None;tuning_changed=False
    report=dict(status='running',target=a.goal,motion_profile=a.motion_profile,policy=policy,
        landmarks=landmarks,reached=[],speed_contexts=[],feedback_recoveries=0,nav2_retries=[],
        grip_commands_sent=False,goal_precision_changed=False,parameters_changed=False,
        joint_staleness_alone_cancels_navigation=False,continuous_landmark_preemption=True)
    subs=[node.create_subscription(kind,topic,lambda m,key=key:data.update({key:m}),qos_profile_sensor_data)
        for key,kind,topic in (('joints',JointState,'/hsrb/joint_states'),('scan',LaserScan,'/hsrb/base_scan'))]
    client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    getter=node.create_client(GetParameters,'/controller_server/get_parameters')
    setter=node.create_client(SetParametersAtomically,'/controller_server/set_parameters_atomically')
    state_client=node.create_client(GetState,'/controller_server/get_state')
    manager=node.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
    def save():
        report['elapsed_s']=time.monotonic()-started
        tmp=a.output.with_suffix('.tmp');tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2));tmp.replace(a.output)
    def event(e):
        report.setdefault('events',[]).append(dict(at_s=time.monotonic()-started,**e));save()
        print(json.dumps(e,ensure_ascii=False),flush=True)
    recovery=RecoveryStatus(a.output.parent,event)
    def pump(dt=.02):rclpy.spin_once(node,timeout_sec=dt)
    def wait(future,timeout=5.):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:pump()
        if not future.done():raise RuntimeError('navigation_service_or_goal_response_timeout')
        if future.exception():raise RuntimeError(str(future.exception()))
        return future.result()
    def read_params(names):
        if not getter.wait_for_service(timeout_sec=5):raise RuntimeError('controller_parameter_service_unavailable')
        req=GetParameters.Request();req.names=list(names);values=wait(getter.call_async(req)).values
        if len(values)!=len(names) or any(v.type!=3 for v in values):raise ValueError('invalid_controller_parameter_response')
        return [v.double_value for v in values]
    def write_params(names,values):
        if not setter.wait_for_service(timeout_sec=5):raise RuntimeError('atomic_controller_parameter_service_unavailable')
        req=SetParametersAtomically.Request();req.parameters=[Parameter(k,value=float(v)).to_parameter_msg() for k,v in zip(names,values)]
        result=wait(setter.call_async(req)).result
        if not result.successful:raise RuntimeError('controller_parameters_rejected:'+result.reason)
    speed_lease=NavigationSpeedLease(lambda:read_params(NAV_SPEED_NAMES),lambda vals:write_params(NAV_SPEED_NAMES,vals))
    def lifecycle(cmd):
        if not manager.wait_for_service(timeout_sec=5):raise RuntimeError('navigation_manager_unavailable')
        req=ManageLifecycleNodes.Request();req.command=cmd
        if not wait(manager.call_async(req),30).success:raise RuntimeError('navigation_lifecycle_failed')
    def idle():
        if not state_client.wait_for_service(timeout_sec=5):raise RuntimeError('controller_state_unavailable')
        if wait(state_client.call_async(GetState.Request())).current_state.id!=1:lifecycle(3)
    def ownership():
        # Arm freshness is advisory only while no trajectory controller owns it.
        for topic in ('/hsrb/omni_base_controller/command','/hsrb/arm_trajectory_controller/command','/hsrb/gripper_controller/command'):
            if node.count_publishers(topic):raise ValueError('conflicting_motion_controller:'+topic)
    def check(*,initial=False):
        nonlocal last_advisory
        for _ in range(20):pump(0.)
        ownership()
        if 'scan' not in data or 'joints' not in data:raise FeedbackPending('missing_navigation_feedback')
        now=node.get_clock().now().nanoseconds
        ages={k:(now-Time.from_msg(data[k].header.stamp).nanoseconds)/1e9 for k in ('scan','joints')}
        try:tf=buffer.lookup_transform('map','base_footprint',Time())
        except Exception as e:
            if type(e).__name__ in ('LookupException','ConnectivityException','ExtrapolationException'):raise FeedbackPending('missing_navigation_tf') from e
            raise
        ages['tf']=(now-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
        j=dict(zip(data['joints'].name,data['joints'].position))
        names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        feedback=feedback_status(ages,[j.get(k) for k in names],policy,initial=initial)
        t=tf.transform.translation;q=tf.transform.rotation
        pose=[t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        if not all(math.isfinite(v) for v in pose):raise ValueError('invalid_navigation_pose')
        report['last_verified_feedback']=dict(pose=pose,ages_s=ages,checked_ns=now,
            stamps_ns=dict(scan=Time.from_msg(data['scan'].header.stamp).nanoseconds,tf=Time.from_msg(tf.header.stamp).nanoseconds))
        if feedback['joints_advisory_stale'] and time.monotonic()-last_advisory>=5:
            event(dict(event='joint_feedback_advisory',age_s=ages['joints'],navigation_cancelled=False));last_advisory=time.monotonic()
        return pose
    def ready(*,initial=False,stopped=False):
        anchor=None;since=None;stamps=None
        while True:
            try:
                pose=check(initial=initial)
                if not stopped:recovery.recovered();return pose
                current=report['last_verified_feedback']['stamps_ns']
                if anchor is None or math.dist(anchor[:2],pose[:2])>.01 or abs(wrap(anchor[2]-pose[2]))>.02:
                    anchor=pose;since=time.monotonic();stamps=current
                elif time.monotonic()-since>=.2 and all(current[k]>stamps[k] for k in ('scan','tf')):
                    recovery.recovered();return pose
            except FeedbackPending as e:
                recovery.waiting(str(e));anchor=None
            pump()
    def cancel_all():
        for handle,future in handles:
            if not future.done():wait(handle.cancel_goal_async())
        for handle,future in handles:
            result=wait(future)
            if result.status not in (4,5,6):raise RuntimeError('navigation_stop_not_confirmed')
        event(dict(event='all_owned_navigation_goals_terminal'))
    def apply_caps(pose,leg_distance):
        nonlocal last_caps
        caps,zone=caps_for_pose(profile,pose,leg_distance,doors,a.goal,policy,holding=.02<a.holding_opening<.62)
        if caps!=last_caps:
            speed_lease.apply(caps);last_caps=caps
            row=dict(at_s=time.monotonic()-started,pose=pose,zone=zone,requested_caps=caps)
            report['speed_contexts'].append(row);save()
    def send(row):
        check();goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x=row['pose'][0];goal.pose.pose.position.y=row['pose'][1]
        goal.pose.pose.orientation.z=math.sin(row['pose'][2]/2);goal.pose.pose.orientation.w=math.cos(row['pose'][2]/2)
        goal.behavior_tree=str(Path(__file__).parent/'config/navigate_efficiency_v1.xml')
        handle=wait(client.send_goal_async(goal))
        if not handle.accepted:raise RuntimeError('navigation_goal_rejected')
        future=handle.get_result_async();handles.append((handle,future));return handle,future
    def front_distance():
        scan=data.get('scan')
        if scan is None:return None
        vals=[v for i,v in enumerate(scan.ranges) if abs(wrap(scan.angle_min+i*scan.angle_increment))<.5 and math.isfinite(v) and v>=scan.range_min]
        return min(vals) if vals else None
    try:
        # Readiness, not a fixed four-second startup sleep.
        report['start']=ready(initial=True)
        repair_navigation(node,root=a.output.parent,record=event)
        idle()
        if not precision_matches(read_params(PRECISION_NAMES)):raise ValueError('final_navigation_precision_not_ready')
        report['current_precision']=read_params(PRECISION_NAMES)
        tuning_names=list(policy['controller_parameters']);tuning_values=list(policy['controller_parameters'].values())
        tuning_before=read_params(tuning_names);tuning_changed=True
        write_params(tuning_names,tuning_values)
        if any(abs(x-y)>1e-8 for x,y in zip(read_params(tuning_names),tuning_values)):raise RuntimeError('navigation_tuning_readback_failed')
        # Configure AFTER writes so cached DWB acceleration/critic values agree.
        lifecycle(0)
        while not client.wait_for_server(timeout_sec=.5):recovery.waiting('waiting_for_navigation_action_server');pump()
        recovery.recovered();report['startup_ready_s']=time.monotonic()-started;save()
        index=0;current=None;retry=RetryState(policy);leg_distance=0.
        while index<len(landmarks):
            row=landmarks[index]
            try:
                pose=check()
                if current is None:
                    leg_distance=math.dist(pose[:2],row['pose'][:2]);apply_caps(pose,leg_distance)
                    current=send(row);event(dict(event='landmark_goal_accepted',index=index,kind=row['kind'],pose=row['pose']))
                pump(.02);pose=check();apply_caps(pose,leg_distance)
                assessment=pass_assessment(row,pose,policy)
                if row['kind']!='final' and assessment['passed']:
                    report['reached'].append(dict(index=index,target=row['pose'],actual=pose,kind=row['kind'],status='passed_in_motion',assessment=assessment))
                    event(dict(event='landmark_passed_without_cancel_or_stop',index=index,kind=row['kind']))
                    # Keep the previous action active until the next accepted
                    # goal replaces it in Nav2's own preemption machinery.
                    index+=1;current=None;retry=RetryState(policy);continue
                handle,future=current
                if not future.done():continue
                result=future.result();pose=check()
                error=math.dist(pose[:2],row['pose'][:2]);yaw=abs(wrap(pose[2]-row['pose'][2]))
                if result.status==4:
                    if row['kind']=='final' and (error>policy['final_acceptance'][0] or yaw>policy['final_acceptance'][1]):
                        reason='final_pose_requires_reacquisition_or_correction'
                    elif row['kind']!='final' and not pass_assessment(row,pose,policy)['passed']:
                        reason='landmark_not_physically_passed_reacquire_and_retry'
                    else:
                        report['reached'].append(dict(index=index,target=row['pose'],actual=pose,error_m=error,yaw_error_rad=yaw,status=4,kind=row['kind']))
                        report.update(action_status=4,final_pose=pose,position_error_m=error)
                        index+=1;current=None;retry=RetryState(policy);save();continue
                elif result.status==6:reason='nav2_local_recovery_exhausted'
                else:raise RuntimeError('navigation_cancelled_or_unknown_terminal_status:'+str(result.status))
                pending=retry.failed(pose,front_obstacle_m=front_distance());pending.update(reason=reason,index=index)
                report['nav2_retries'].append(pending);event(dict(event='navigation_task_retained',**pending))
                until=time.monotonic()+pending['wait_s']
                while time.monotonic()<until:
                    ready();pump(min(.05,max(0.,until-time.monotonic())))
                current=None
            except FeedbackPending as exc:
                # Base feedback loss always cancels and confirms a real stop;
                # stale joints alone never enter this path during navigation.
                report['feedback_recoveries']+=1;event(dict(event='base_feedback_stop_and_recover',reason=str(exc),index=index))
                cancel_all();ready(stopped=True);current=None
        # Current goal completion precedes final state/parameter handoff.
        cancel_all();ready(initial=True);report['status']='arrived'
    except (Exception,KeyboardInterrupt) as exc:
        report.update(status='failed',error=str(exc) or 'interrupted');report.pop('final_pose',None)
        try:cancel_all()
        except Exception as e:report['cancel_error']=str(e)
    finally:
        try:idle();report['nav_reset']=True
        except Exception as e:report.update(nav_reset=False,nav_reset_error=str(e),status='failed')
        try:speed_lease.restore();report['speed_parameters_restored']=True
        except Exception as e:report.update(speed_parameters_restored=False,speed_restore_error=str(e),status='failed')
        if tuning_changed:
            try:
                write_params(tuning_names,tuning_before)
                if any(abs(x-y)>1e-8 for x,y in zip(read_params(tuning_names),tuning_before)):raise RuntimeError('tuning_restore_readback_failed')
                report['tuning_parameters_restored']=True
            except Exception as e:report.update(tuning_parameters_restored=False,tuning_restore_error=str(e),status='failed')
        report.update(parameters_changed=bool(tuning_changed or speed_lease.changed),speed_parameters_changed=speed_lease.changed)
        save();print(json.dumps(report,ensure_ascii=False),flush=True);node.destroy_node();rclpy.shutdown()
    if report['status']!='arrived':raise SystemExit(1)

if __name__=='__main__':main()
