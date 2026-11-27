"""Angle-binned moving RGBD acquisition. Stationary grasp capture is unchanged."""
import math,time
from concurrent.futures import ThreadPoolExecutor
import numpy as np

def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))
def bin_index(yaw,start,count=12):return int(math.floor(((yaw-start)%(2*math.pi))/(2*math.pi/count)+.5))%count

def covered_bin(yaw,start,count=12):
    idx=bin_index(yaw,start,count);step=2*math.pi/count
    return idx if abs(wrap(yaw-start-idx*step))<=step*.25 else None

class SweepProgress:
    def __init__(self,yaw):self.previous=yaw;self.travel=0.
    def update(self,yaw):
        delta=wrap(yaw-self.previous);self.previous=yaw
        if abs(delta)>.6:raise ValueError('continuous_scan_pose_jump')
        self.travel+=delta
        return self.travel

class ContinuousSweep:
    def __init__(self,search):
        self.s=search;self.rt=search.rt;self.opts=search.options
        self.speed=float(self.opts.get('continuous_speed_rad_s',.4));self.count=int(self.opts.get('continuous_bins',12))
        if not 0<self.speed<=.6 or self.count<12:raise ValueError('invalid_continuous_scan_settings')
    def heading(self):
        from rclpy.time import Time
        tf=self.rt.buf.lookup_transform('map','base_footprint',Time())
        age=self.rt.node.get_clock().now().nanoseconds-self.rt.stamp(tf)
        self.last_pose_age_ns=age
        if age< -20_000_000 or age>500_000_000:raise RuntimeError('continuous_scan_stale_pose')
        q=tf.transform.rotation;return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def wait_heading(self,cancel,zero):
        waiting=None
        while True:
            self.s.check(cancel)
            try:
                yaw=self.heading()
            except RuntimeError as exc:
                if str(exc)!='continuous_scan_stale_pose':raise
                zero()
                if waiting is None:
                    waiting=time.monotonic()
                    self.rt.record(dict(event='continuous_scan_pose_wait',
                        pose_age_ns=getattr(self,'last_pose_age_ns',None)))
                self.rt.pump(.05)
                continue
            if waiting is not None:
                self.rt.record(dict(event='continuous_scan_pose_recovered',
                    waited_s=time.monotonic()-waiting,pose_age_ns=getattr(self,'last_pose_age_ns',None)))
            return yaw

    def acquire(self):
        # Crucial: use image-time TF, NOT the default stationary/latest TF.
        from profiled_capture import acquire
        from rclpy.time import Time
        item=acquire(self.rt,'rgbd',timeout=.25,time_factory=lambda ns:Time(nanoseconds=ns))
        snap=item['snapshot'];snap['transform_basis']='image_timestamp_during_continuous_scan'
        for tf in snap['tracking_transforms'].values():tf['basis']='image_timestamp'
        snap['motion_authorized']=False
        return item
    def run(self,target,cancel,label,*,scan_start=None,remaining_angle=None,skip_bins=()):
        from geometry_msgs.msg import Twist
        turn_guard=getattr(self.s.motion,'turn_guard',None) if hasattr(self.s,'motion') else None
        clearance=turn_guard.prepare_sweep() if turn_guard is not None else None
        start=None;progress=None;step=2*math.pi/self.count
        topic='/hsrb/command_velocity';self.rt.ownership()
        pub=self.rt.node.create_publisher(Twist,topic,10);timer=None;pending=[];rows=[];submitted=set(skip_bins);pool=ThreadPoolExecutor(max_workers=1)
        enabled=False;paused=[False];fault=[];last_progress=time.monotonic();previous_travel=0.;deadline=time.monotonic()+5
        def zero():pub.publish(Twist())
        def tick():
            if not enabled or paused[0] or cancel.is_set():zero();return
            try:
                self.s.check(cancel);self.heading();self.rt.ownership(topic)
                if clearance is not None:
                    from rclpy.time import Time
                    tf=self.rt.buf.lookup_transform('map','base_footprint',Time())
                    t=tf.transform.translation;pose=[t.x,t.y,self.heading()]
                    if not clearance.turn_path(pose,self.speed*.5)['clear']:raise ValueError('continuous_scan_turn_blocked')
                msg=Twist();msg.angular.z=self.speed;pub.publish(msg)
            except Exception as exc:
                if isinstance(exc,RuntimeError) and str(exc)=='continuous_scan_stale_pose':
                    paused[0]=True
                    self.rt.record(dict(event='continuous_scan_pose_pause',pose_age_ns=getattr(self,'last_pose_age_ns',None)))
                else:fault.append(str(exc))
                zero()
        def collect():
            for future,item,path,idx in list(pending):
                if not future.done():continue
                row=self.s.evaluate(item,path,future.result(),target);row.update(angle_bin=idx,scan_start_yaw=start)
                rows.append(row);pending.remove((future,item,path,idx))
        try:
            while pub.get_subscription_count()!=1:
                self.s.check(cancel)
                if time.monotonic()>deadline:raise RuntimeError('scan_velocity_receiver_unavailable')
                self.rt.pump(.02)
            heading=self.wait_heading(cancel,zero);start=heading if scan_start is None else scan_start;progress=SweepProgress(heading)
            limit=2*math.pi-.03 if remaining_angle is None else max(0.,remaining_angle)
            last_progress=time.monotonic()
            self.rt.ownership(topic);timer=self.rt.node.create_timer(.05,tick);enabled=True
            self.rt.record(dict(event='continuous_scan_started',speed_rad_s=self.speed,bins=self.count,head_tilt=self.opts.get('head_tilts'),label=label))
            while progress.travel<limit:
                self.s.check(cancel);self.rt.pump(.01)
                if fault:raise RuntimeError(fault[0])
                if paused[0]:
                    # Exclude unobserved coasting from covered sweep angle, retain
                    # submitted bins/results; missed bins are rechecked by caller.
                    progress.previous=self.wait_heading(cancel,zero)
                    last_progress=time.monotonic();paused[0]=False
                    collect();continue
                try:yaw=self.heading()
                except RuntimeError as exc:
                    if str(exc)!='continuous_scan_stale_pose':raise
                    paused[0]=True;zero();continue
                travel=progress.update(yaw);collect()
                if travel>previous_travel+.01:previous_travel=travel;last_progress=time.monotonic()
                if time.monotonic()-last_progress>5:raise RuntimeError('continuous_scan_rotation_not_progressing')
                idx=bin_index(yaw,start,self.count)
                # Bound inference backlog. Missed bins are explicitly revisited after the sweep.
                if idx not in submitted and len(pending)<2:
                    try:item=self.acquire()
                    except RuntimeError as exc:
                        if str(exc).startswith('observation_slice_pending:'):continue
                        raise
                    q=item['snapshot']['tracking_transforms']['map_from_base_at_depth']['quaternion']
                    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]));idx=covered_bin(yaw,start,self.count)
                    if idx is None or idx in submitted:continue
                    path=self.rt.save(item,f'{label}-bin{idx:02d}')
                    future=pool.submit(self.s.b['worker'].infer,path,pump=lambda:None)
                    pending.append((future,item,path,idx));submitted.add(idx)
                # A strong candidate triggers stationary confirmation, not a moving-frame grasp.
                if any(row.get('candidates') or row['matches'] or row['unresolved'] for row in rows):break
        finally:
            enabled=False
            if timer is not None:self.rt.node.destroy_timer(timer)
            for _ in range(3):zero();self.rt.pump(.02)
            self.rt.node.destroy_publisher(pub)
            pool.shutdown(wait=True)
        self.s.check(cancel);collect()
        self.rt.sample(profile='state') # Confirm stopped before head/arm or navigation commands.
        self.rt.record(dict(event='continuous_scan_finished',label=label,travel_rad=progress.travel,frames=len(rows)))
        return start,rows,progress.travel
