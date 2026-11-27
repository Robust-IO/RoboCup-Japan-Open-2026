"""One finite Unity odom trajectory at the reviewed table pose.

Dry run by default. No Twist publisher, no auto retry, no cancellation claim.
Uses the previously reviewed tucked-arm envelope; planar checks are not a
general 3-D collision certificate. This is a bounded simulation trial only.
"""
import argparse
import json
import math
import os
import time
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import JointState,LaserScan
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from tf2_ros import Buffer,TransformListener,TransformException,ExtrapolationException
from hand_camera_ray import rotation
from finite_base_step_plan import plan,remaining_translation
from fresh_feedback_wait import wait_for_fresh
from aligned_scan import select_aligned
from feedback_recovery import Recovery

TOPIC='/hsrb/omni_base_controller/command'


def parts(tf):
    t=tf.transform.translation;q=tf.transform.rotation
    xyz=np.array([t.x,t.y,t.z]);r=rotation([q.x,q.y,q.z,q.w])
    if not np.isfinite(xyz).all():raise ValueError('invalid_tf')
    return xyz,r


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--monitor-only',action='store_true');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reviewed-map-anchor',type=float,nargs=2,default=[3.1743,.3133])
    p.add_argument('--profile',choices=['tucked_open_lowview','tucked_closed_tableview','tucked_closed_lowview','extended_open_060','tucked_holding'],default='tucked_open_lowview')
    p.add_argument('--distance',type=float,default=.015)
    p.add_argument('--simulation-contact-trial',action='store_true',help='User-authorized simulation experiment: up to 10cm, occupied map cells warn only; unknown cells still block.')
    p.add_argument('--simulation-large-step',action='store_true',help='Explicit simulation-only opt-in, maximum 35mm; default remains 15mm.')
    p.add_argument('--direction',type=float,nargs=2,default=[.09336279355,-.0299586638]);a=p.parse_args()
    if not all(math.isfinite(v) for v in a.direction) or not ((0<a.direction[0]<=.1) or (a.simulation_contact_trial and -.25<=a.direction[0]<0)) or abs(a.direction[1])>.04:raise ValueError('Invalid reviewed direction')
    def proposal_for(pose):return plan(pose,direction=a.direction,distance=a.distance,simulation_large_step=a.simulation_large_step,simulation_contact_trial=a.simulation_contact_trial)
    proposal_for((0,0,0))
    if a.profile=='extended_open_060' and not a.simulation_contact_trial:raise ValueError('Extended pose requires explicit simulation contact trial')
    if not all(math.isfinite(v) for v in a.reviewed_map_anchor) or math.dist(a.reviewed_map_anchor,[3.1743,.3133])>(.3 if a.simulation_contact_trial else .1):raise ValueError('Outside reviewed table neighborhood')
    if a.run and a.monitor_only:raise ValueError('Read-only monitor cannot use --run')
    if os.environ.get('ROS_DOMAIN_ID')!='71':raise ValueError('Requires reviewed simulation domain 71')
    a.output.mkdir(exist_ok=False)
    rclpy.init();n=rclpy.create_node('finite_base_single_trial');b=Buffer();listener=TransformListener(b,n);data={};pub=None
    scans=deque(maxlen=20);alignment_stats=dict(checks=0,used_previous=0,max_scan_age_s=0.)
    arrivals={'scan':[],'joints':[],'tf':[]}
    def record(kind,stamp):
        if a.monitor_only:arrivals[kind].append(dict(received=time.monotonic(),age_s=(n.get_clock().now()-Time.from_msg(stamp)).nanoseconds/1e9))
    def receive_scan(m):scans.append(m);data.update(scan=m);record('scan',m.header.stamp)
    def receive_joints(m):data.update(joints=m);record('joints',m.header.stamp)
    def receive_tf(m):
        for t in m.transforms:
            if t.child_frame_id=='base_footprint':record('tf',t.header.stamp)
    subs=[n.create_subscription(JointState,'/hsrb/joint_states',receive_joints,qos_profile_sensor_data),
          n.create_subscription(LaserScan,'/hsrb/base_scan',receive_scan,qos_profile_sensor_data),
          n.create_subscription(OccupancyGrid,'/map',lambda m:data.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))]
    if a.monitor_only:subs.append(n.create_subscription(TFMessage,'/tf',receive_tf,qos_profile_sensor_data))
    events=[];report=dict(command_sent=False,automatic_retry=False,cancellation_supported=False,reviewed_map_anchor=a.reviewed_map_anchor)
    report.update(profile=a.profile,direction=a.direction)
    report['simulation_contact_trial']=a.simulation_contact_trial
    report['occupied_map_warning_samples']=0
    def age(stamp):return (n.get_clock().now()-Time.from_msg(stamp)).nanoseconds/1e9
    def fresh(stamp):
        if not -.02<=age(stamp)<=.4:raise RuntimeError('stale_capture')
    def state():
        if len(data)!=3:raise RuntimeError('missing_streams')
        j=data['joints'];fresh(j.header.stamp);fresh(data['scan'].header.stamp)
        expected=dict(arm_lift_joint=0.,arm_flex_joint=0.,arm_roll_joint=0.,wrist_flex_joint=-math.pi/2,
                      wrist_roll_joint=0.,hand_motor_joint=.65,head_pan_joint=0.,head_tilt_joint=-.8)
        if a.profile=='tucked_closed_tableview':expected.update(hand_motor_joint=0.,head_tilt_joint=-.5)
        if a.profile=='tucked_closed_lowview':expected.update(hand_motor_joint=0.,head_tilt_joint=-.8)
        if a.profile=='extended_open_060':expected.update(arm_lift_joint=.3228021848,arm_flex_joint=-1.7,wrist_flex_joint=.1292036732)
        if a.profile=='tucked_holding':expected.update(hand_motor_joint=.4122018066)
        if len(j.name)!=len(j.position):raise RuntimeError('invalid_joints')
        for name,value in expected.items():
            if j.name.count(name)!=1:raise RuntimeError('missing_joint')
            actual=j.position[j.name.index(name)]
            if not math.isfinite(actual) or abs(actual-value)>.005:raise RuntimeError('pose_not_screened')
        for topic in ('/hsrb/command_velocity','/hsrb/arm_trajectory_controller/command','/hsrb/gripper_controller/command','/hsrb/head_trajectory_controller/command'):
            if n.count_publishers(topic):raise RuntimeError('other_controller_present:'+topic)
        if n.count_publishers(TOPIC)!=(1 if pub else 0) or n.count_subscribers(TOPIC)!=1:raise RuntimeError('omni_control_not_exclusive')
        tf=b.lookup_transform('odom','base_footprint',Time());fresh(tf.header.stamp);xyz,r=parts(tf)
        mapped=b.lookup_transform('map','base_footprint',Time());fresh(mapped.header.stamp);m,mr=parts(mapped)
        yaw=math.atan2(r[1,0],r[0,0]);myaw=math.atan2(mr[1,0],mr[0,0])
        if np.linalg.norm(m[:2]-a.reviewed_map_anchor)>max(.025,a.distance+.01) or abs(myaw+1.393)>.02:raise RuntimeError('outside_reviewed_staging')
        return (float(xyz[0]),float(xyz[1]),yaw),tf.header.stamp
    def clearance(target_odom=None,stationary=False):
        def available(scan):
            stamp=Time.from_msg(scan.header.stamp)
            return b.can_transform('base_footprint',scan.header.frame_id,stamp) and b.can_transform(data['map'].header.frame_id,'base_footprint',stamp) and (target_odom is None or b.can_transform('odom','base_footprint',stamp))
        scan=select_aligned(scans,n.get_clock().now().nanoseconds,lambda m:Time.from_msg(m.header.stamp).nanoseconds,available)
        fresh(scan.header.stamp);stamp=Time.from_msg(scan.header.stamp)
        st,sr=parts(b.lookup_transform('base_footprint',scan.header.frame_id,stamp))
        ranges=np.asarray(scan.ranges);angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        good=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)
        if good.sum()<100:raise RuntimeError('insufficient_scan')
        points=np.c_[ranges[good]*np.cos(angles[good]),ranges[good]*np.sin(angles[good]),np.zeros(good.sum())]@sr.T+st
        delta=np.array(proposal_for((0,0,0))['relative_base_translation_m'])
        if stationary:delta=np.zeros(2)
        elif target_odom is not None:
            ot_pose,orr=parts(b.lookup_transform('odom','base_footprint',stamp))
            scan_pose=(float(ot_pose[0]),float(ot_pose[1]),math.atan2(orr[1,0],orr[0,0]))
            delta=np.array(remaining_translation(scan_pose,target_odom))
            if np.linalg.norm(delta)>a.distance+.01:raise RuntimeError('unexpected_remaining_distance')
        centers=np.linspace(0,1,max(2,math.ceil(float(np.linalg.norm(delta))/.002)+1))[:,None]*delta
        nearest=float(np.linalg.norm(points[None,:,:2]-centers[:,None,:],axis=2).min())
        if nearest<.43:raise RuntimeError('laser_obstacle')
        grid=data['map'];res=grid.info.resolution
        if not 0<res<=.1:raise RuntimeError('invalid_map_resolution')
        g=np.asarray(grid.data).reshape(grid.info.height,grid.info.width)
        mt,mr=parts(b.lookup_transform(grid.header.frame_id,'base_footprint',stamp))
        o=grid.info.origin;q=o.orientation;ro=rotation([q.x,q.y,q.z,q.w]);ot=np.array([o.position.x,o.position.y,o.position.z])
        gc=(np.c_[centers,np.zeros(len(centers))]@mr.T+mt-ot)@ro
        cells=int(math.ceil(.41/res))+1
        for x,y,_ in gc:
            cx=int(math.floor(x/res));cy=int(math.floor(y/res));yy,xx=np.mgrid[cy-cells:cy+cells+1,cx-cells:cx+cells+1]
            mask=np.hypot((xx+.5)*res-x,(yy+.5)*res-y)<=.41+res/math.sqrt(2)
            inside=(xx>=0)&(yy>=0)&(xx<grid.info.width)&(yy<grid.info.height)
            if np.any(mask&~inside):raise RuntimeError('outside_map')
            vals=g[yy[mask],xx[mask]]
            if np.any(vals<0):raise RuntimeError('map_unknown')
            if np.any(vals>=65):
                if not a.simulation_contact_trial:raise RuntimeError('map_blocked_or_unknown')
                report['occupied_map_warning_samples']+=1
        fresh(scan.header.stamp)
        alignment_stats['checks']+=1
        alignment_stats['used_previous']+=int(scan is not data['scan'])
        alignment_stats['max_scan_age_s']=max(alignment_stats['max_scan_age_s'],age(scan.header.stamp))
        return dict(laser_nearest_m=nearest,map_samples=len(centers),scan_age_s=age(scan.header.stamp),checked_translation_m=delta.tolist(),fixed_endpoint=target_odom is not None,stationary=stationary)
    def ready_clearance():
        deadline=time.monotonic()+1
        while True:
            try:return clearance()
            except (TransformException,RuntimeError) as e:
                if isinstance(e,RuntimeError) and not isinstance(e,TransformException) and str(e) not in ('stale_capture','no_fresh_aligned_scan'):raise
                if time.monotonic()>=deadline:raise RuntimeError('clearance_tf_wait_expired') from e
                rclpy.spin_once(n,timeout_sec=.01)
    try:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
        if a.monitor_only:
            started=time.monotonic();deadline=started+20;failures={};valid=0;episodes=[];active=None;recovery=Recovery()
            while time.monotonic()<deadline:
                rclpy.spin_once(n,timeout_sec=.01)
                try:
                    _,stamp=state();clearance(stationary=True);valid+=1
                    recovery.good(time.monotonic(),Time.from_msg(stamp).nanoseconds)
                    if active is not None:
                        active.update(recovered=True,duration_s=time.monotonic()-active['start_monotonic']);episodes.append(active);active=None
                except Exception as e:
                    failures[str(e)]=failures.get(str(e),0)+1
                    now=time.monotonic()
                    recovery.bad(now,'tf_not_ready' if isinstance(e,ExtrapolationException) else str(e))
                    if active is None:active=dict(start_monotonic=now,elapsed_s=now-started,details=[])
                    detail=dict(reason=str(e),elapsed_s=now-started,joints_age_s=age(data['joints'].header.stamp),scan_age_s=age(data['scan'].header.stamp))
                    for parent,child,key in [('odom','base_footprint','base_tf_age_s'),('base_footprint',data['scan'].header.frame_id,'laser_tf_age_s')]:
                        try:detail[key]=age(b.lookup_transform(parent,child,Time()).header.stamp)
                        except Exception as exc:detail[key]=str(exc)
                    detail['queued_scan_ages_s']=[age(m.header.stamp) for m in scans]
                    active['details'].append(detail)
            if active is not None:active.update(recovered=False,duration_s=time.monotonic()-active['start_monotonic']);episodes.append(active)
            transport={}
            for kind,rows in arrivals.items():
                gaps=[right['received']-left['received'] for left,right in zip(rows,rows[1:])];ages=[r['age_s'] for r in rows]
                transport[kind]=dict(messages=len(rows),max_gap_s=max(gaps,default=0),max_receive_age_s=max(ages,default=0))
            verified=recovery.verified(time.monotonic())
            report.update(status='current_feedback_verified' if verified else 'monitor_not_verified',valid_checks=valid,failures=failures,episodes=episodes,transport=transport,recovery=recovery.summary(time.monotonic()))
            return
        epoch=[0]
        def gap():epoch[0]+=1
        def ready():return wait_for_fresh(state,lambda dt:rclpy.spin_once(n,timeout_sec=dt),gap)
        anchor,_=ready();samples=[];stable_since=time.monotonic();deadline=stable_since+5
        while time.monotonic()-stable_since<1.2:
            if time.monotonic()>deadline:raise RuntimeError('stationary_window_timeout')
            previous=epoch[0];rclpy.spin_once(n,timeout_sec=.01);pose,stamp=ready();samples.append(pose)
            if epoch[0]!=previous:stable_since=time.monotonic()
            if math.dist(pose[:2],anchor[:2])>.005 or abs(pose[2]-anchor[2])>.005:raise RuntimeError('base_not_stationary')
        report['clearance']=ready_clearance();pose,_=ready();proposal=proposal_for(pose);report['proposal']=proposal
        if not a.run:
            report['status']='dry_run_passed';return
        pub=n.create_publisher(JointTrajectory,TOPIC,10)
        deadline=time.monotonic()+.5
        while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
        pose,_=ready();report['clearance']=ready_clearance();pose,_=state()
        if math.dist(pose[:2],anchor[:2])>.005:raise RuntimeError('base_moved_before_send')
        proposal=proposal_for(pose);report['proposal']=proposal
        msg=JointTrajectory();msg.joint_names=proposal['joint_names'];pt=JointTrajectoryPoint();pt.positions=proposal['positions'];pt.time_from_start.sec=2;msg.points=[pt]
        pub.publish(msg);report['command_sent']=True;sent=time.monotonic();print(f'One finite {a.distance*1000:.1f}mm trajectory sent; no automatic next segment.',flush=True)
        history=[];recovery=Recovery()
        while time.monotonic()-sent<5:
            rclpy.spin_once(n,timeout_sec=.01)
            try:
                current,stamp=state();elapsed=time.monotonic()-sent
                if math.dist(current[:2],pose[:2])>a.distance+.01:raise RuntimeError('unexpected_displacement')
                report['last_motion_clearance']=clearance(target_odom=proposal['positions'])
                recovery.good(time.monotonic(),Time.from_msg(stamp).nanoseconds)
                history.append(dict(elapsed=elapsed,pose=current,stamp_ns=Time.from_msg(stamp).nanoseconds))
            except Exception as e:
                reason='tf_not_ready' if isinstance(e,ExtrapolationException) else str(e)
                recovery.bad(time.monotonic(),reason)
                if not events or events[-1].get('reason')!=reason:events.append(dict(event='monitor_unavailable' if reason in Recovery.transient else 'monitor_fault',reason=reason))
                # Do not send Twist zero: receiver rejects it during this finite trajectory.
                # Continue read-only monitoring; no second trajectory, no fake cancellation.
        recent=[h for h in history if h['elapsed']>=4]
        if not recent:raise RuntimeError('no_final_feedback')
        final=recent[-1]['pose'];spread=max(math.dist(h['pose'][:2],final[:2]) for h in recent)
        report.update(final_odom_pose=final,observed_displacement_m=math.dist(final[:2],pose[:2]),
                      final_target_error_m=math.dist(final[:2],proposal['positions'][:2]),final_position_spread_m=spread,history=history)
        report['recovery']=recovery.summary(time.monotonic())
        if recovery.fault:raise RuntimeError('monitor_failed:'+recovery.fault)
        if not recovery.verified(time.monotonic()):raise RuntimeError('final_feedback_not_reverified')
        if recent[-1]['elapsed']<4.8 or spread>.005 or report['final_target_error_m']>.006:raise RuntimeError('final_pose_not_verified')
        report['status']='endpoint_verified_with_observation_gap' if recovery.gaps else 'passed'
    except Exception as e:report.update(status='failed',reason=str(e))
    finally:
        report['events']=events
        report['alignment_stats']=alignment_stats
        (a.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='history'}))
        n.destroy_node();rclpy.shutdown()
    if report['status']=='failed':raise SystemExit(1)


if __name__=='__main__':main()
