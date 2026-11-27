"""Only navigation velocity publisher; Nav2 publishes to a private input topic.

Created for the lifetime of a navigation child, destroyed before finite motions.
No subscriber that races another publisher on the robot's command topic.
"""
import json,math,time
from collections import deque
from pathlib import Path
import numpy as np
from carry_clearance import CarryClearance
from candidate_selection import CandidateSelector
from scipy.spatial import ConvexHull
from navigation_turn_policy import TurnPolicy,ArrivalHeading,final_heading_command
from navigation_blockage import BlockageWatch

RAW_TOPIC='/handyman/navigation_velocity_unchecked'
OUTPUT_TOPIC='/hsrb/command_velocity'

class TransformMatchedScan:
    """Keep exact-time scan/TF pairs; never project a scan with latest TF."""
    def __init__(self):
        self.pending=deque(maxlen=16);self.matched=None
        self.failures=0;self.last_error=None;self.reused=0
    def add(self,scan):self.pending.append(scan)
    def select(self,now_ns,lookup):
        def stamp(scan):return scan.header.stamp.sec*10**9+scan.header.stamp.nanosec
        def fresh(scan):return -.02<=(now_ns-stamp(scan))/1e9<=.7
        for scan in reversed(self.pending):
            if not fresh(scan):continue
            if self.matched is not None and stamp(scan)<=stamp(self.matched[0]):break
            try:transform=lookup(scan)
            except Exception as exc:
                if type(exc).__name__ not in ('LookupException','ConnectivityException','ExtrapolationException'):raise
                self.failures+=1;self.last_error=type(exc).__name__+': '+str(exc)
                continue
            self.matched=(scan,transform)
            return self.matched
        if self.matched is not None and fresh(self.matched[0]):
            self.reused+=1
            return self.matched
        return None

class NavigationTurnRelay:
    def __init__(self,node,buffer,envelope,goal,record,*,xy_tolerance=.06,xy_acceptance=.085):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Path as NavPath
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data
        self.Twist=Twist;self.node=node;self.buffer=buffer;self.goal=goal;self.record=record
        bundle=np.load(envelope['map_archive'],allow_pickle=False)
        self.selector=CandidateSelector(bundle['free'].astype(bool),float(bundle['resolution']),bundle['origin'],float(bundle['radius']),{})
        self.geometry=CarryClearance(self.selector,np.c_[envelope['polygon'],np.zeros(len(envelope['polygon']))],margin=0)
        self.hull=ConvexHull(self.geometry.polygon).equations
        self.policy=TurnPolicy();self.path=[];self.scan=None;self.latest=None;self.last_time=0.;self.enabled=False;self.error=None;self.last_choice=None;self.waiting_dynamic=False
        self.arrival=ArrivalHeading(xy_tolerance,xy_acceptance)
        self.blockage=BlockageWatch();self.dynamic_reason=None
        self.scan_matcher=TransformMatchedScan()
        self.stats=dict(forwarded=0,stale_input_stops=0,dynamic_wait_ticks=0,tf_wait_ticks=0,directed_turn_ticks=0,max_angular_speed=0.)
        self.pub=node.create_publisher(Twist,OUTPUT_TOPIC,1)
        self.sub=node.create_subscription(Twist,RAW_TOPIC,self.receive,1)
        self.path_sub=node.create_subscription(NavPath,'/plan',lambda m:setattr(self,'path',[(p.pose.position.x,p.pose.position.y) for p in m.poses]),1)
        self.scan_sub=node.create_subscription(LaserScan,'/hsrb/base_scan',self.scan_matcher.add,qos_profile_sensor_data)
        self.timer=node.create_timer(.05,self.tick)
    def zero(self):self.pub.publish(self.Twist())
    def receive(self,msg):self.latest=(msg.linear.x,msg.linear.y,msg.angular.z);self.last_time=time.monotonic()
    def enable(self):
        # A previous launch without remapping must never bypass the check layer.
        peers=self.node.get_publishers_info_by_topic(OUTPUT_TOPIC)
        if any(p.node_name!=self.node.get_name() for p in peers):raise ValueError('navigation_velocity_bypasses_turn_guard_restart_services')
        self.enabled=True
    def pose(self):
        from rclpy.time import Time
        tf=self.buffer.lookup_transform('map','base_footprint',Time())
        age=(self.node.get_clock().now().nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
        if not -.02<=age<=.5:raise ValueError('turn_pose_stale')
        t=tf.transform.translation;q=tf.transform.rotation
        return [t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    def dynamic_clear(self,pose,command):
        """Current laser points can veto a static-map turn (e.g. walking guests)."""
        from rclpy.time import Time
        pair=self.scan_matcher.select(self.node.get_clock().now().nanoseconds,
            lambda scan:self.buffer.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp)))
        self.stats['scan_tf_pending_attempts']=self.scan_matcher.failures
        self.stats['scan_tf_reused_ticks']=self.scan_matcher.reused
        self.stats['scan_tf_last_error']=self.scan_matcher.last_error
        self.dynamic_reason=None
        if pair is None:
            self.dynamic_reason='scan_unavailable'
            self.stats['scan_tf_unavailable_ticks']=self.stats.get('scan_tf_unavailable_ticks',0)+1
            return False
        scan,tf=pair
        q=tf.transform.rotation;t=tf.transform.translation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        ranges=np.asarray(scan.ranges);angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment+yaw
        valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)
        points=np.c_[ranges[valid]*np.cos(angles[valid])+t.x,ranges[valid]*np.sin(angles[valid])+t.y]
        def inside(p):
            c,s=math.cos(p[2]),math.sin(p[2]);local=(points-np.asarray(p[:2]))@np.array([[c,-s],[s,c]])
            return (local@self.hull[:,:2].T+self.hull[:,2]<=.02).all(axis=1)
        # Laser returns already inside the current envelope may be self returns;
        # the static map check still rejects existing furniture overlaps.
        current=inside(pose);vx,vy,w=command;c,s=math.cos(pose[2]),math.sin(pose[2])
        for dt in np.linspace(.1,.5,5):
            future=[pose[0]+dt*(c*vx-s*vy),pose[1]+dt*(s*vx+c*vy),pose[2]+dt*w]
            if (inside(future)&~current).any():
                self.dynamic_reason='obstacle';return False
        return True
    def tick(self):
        if not self.enabled or self.latest is None or time.monotonic()-self.last_time>.3:
            if hasattr(self,'blockage'):self.blockage.update(time.monotonic(),[0,0,0],False)
            self.stats['stale_input_stops']+=1;self.zero();return
        try:
            pose=self.pose()
            # Stop before potentially expensive full-arc comparison. Subsequent
            # latched-turn checks use only the next 0.25rad swept segment.
            target,phase_changed=self.arrival.select(pose,self.goal,self.path,controller_rotating=math.hypot(*self.latest[:2])<.01 and abs(self.latest[2])>1e-6)
            from door_approach_heading import select_heading
            target=select_heading(target,pose,self.goal,getattr(self,'door_approach',False))
            if phase_changed:
                self.zero();self.policy.reset()
                self.record(dict(event='navigation_heading_phase',phase='final_yaw' if self.arrival.arrived else 'approach_xy',
                    distance_m=math.dist(pose[:2],self.goal[:2]),xy_tolerance_m=self.arrival.xy_tolerance))
            if (math.hypot(*self.latest[:2])<.01 and abs(math.atan2(math.sin(target-pose[2]),math.cos(target-pose[2])))>=.15
                    and abs(self.latest[2])>1e-6 and self.policy.remaining is None and not self.policy.can_reuse(pose,target,self.latest[2])):self.zero()
            requested=final_heading_command(pose,self.goal) if self.arrival.arrived else self.latest
            from door_approach_heading import approach_command
            door_command=approach_command(pose,self.goal,getattr(self,'door_approach',False))
            if door_command is not None:requested=door_command
            command=self.policy.step(self.geometry,pose,requested,target)
            if self.policy.last_choice is not self.last_choice:
                self.last_choice=self.policy.last_choice
                self.record(dict(event='navigation_turn_direction',pose=pose,choice=self.last_choice))
            self.waiting_dynamic=any(abs(v)>1e-6 for v in command) and not self.dynamic_clear(pose,command)
            if not hasattr(self,'blockage'):self.blockage=BlockageWatch()
            persistent=self.blockage.update(time.monotonic(),pose,self.waiting_dynamic and getattr(self,'dynamic_reason',None)=='obstacle')
            if self.waiting_dynamic:
                self.stats['dynamic_wait_ticks']+=1;self.zero()
                if persistent:
                    self.error='carry_replan_required:dynamic_sweep_blocked'
                    self.record(dict(event='persistent_velocity_veto_replan',pose=pose,wait_s=self.blockage.wait_s,requested=list(command)))
                return
            if time.monotonic()-self.last_time>.3:
                self.zero();return
            self.stats['forwarded']+=1
            self.stats['directed_turn_ticks']+=int(self.policy.remaining is not None)
            self.stats['max_angular_speed']=max(self.stats['max_angular_speed'],abs(command[2]))
            msg=self.Twist();msg.linear.x,msg.linear.y,msg.angular.z=map(float,command);self.pub.publish(msg)
        except ValueError as exc:
            self.zero()
            if str(exc)=='no_clear_carry_turn_arc':self.error='carry_replan_required:turn_sweep_blocked'
            elif str(exc)!='turn_pose_stale':self.error=str(exc)
        except Exception as exc:
            self.zero()
            if type(exc).__name__ in ('LookupException','ConnectivityException','ExtrapolationException'):self.stats['tf_wait_ticks']+=1;self.stats['tf_last_error']=type(exc).__name__+': '+str(exc)
            else:
                self.error='turn_guard_feedback:'+str(exc)
    def close(self):
        self.enabled=False;self.zero();self.node.destroy_timer(self.timer)
        for sub in (self.sub,self.path_sub,self.scan_sub):self.node.destroy_subscription(sub)
        self.node.destroy_publisher(self.pub)
