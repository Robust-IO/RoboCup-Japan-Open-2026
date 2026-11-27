"""Pure single-step controller. No ROS publishers; stopping is terminal.

The adapter must supply live collision/ownership/posture checks and publish the
returned zero command on rejection. This module alone does not authorize motion.
"""
import math


class BaseStep:
    def __init__(self, start, delta, now):
        if len(start)!=3 or len(delta)!=2 or not all(math.isfinite(v) for v in (*start,*delta,now)):
            raise ValueError('invalid_step')
        distance=math.hypot(*delta)
        if not 0<distance<=.020000001:raise ValueError('step_exceeds_2cm')
        self.start=tuple(start);self.delta=tuple(delta);self.begun=now
        self.last_time=now;self.last_pose=tuple(start);self.terminal=None

    def stop(self, reason):
        self.terminal=reason
        return dict(vx=0.,vy=0.,wz=0.,state=reason)

    def update(self, pose, now, *, tf_age, scan_age, joints_age,
               collision_clear, exclusive, tucked):
        if self.terminal:return self.stop(self.terminal)
        if len(pose)!=3 or not all(math.isfinite(v) for v in (*pose,now,tf_age,scan_age,joints_age)):
            return self.stop('invalid_feedback')
        if now<self.last_time:return self.stop('clock_reversed')
        if now-self.last_time>.2:return self.stop('control_loop_gap')
        if now-self.begun>5:return self.stop('step_deadline')
        if any(not -.02<=age<=.4 for age in (tf_age,scan_age,joints_age)):
            return self.stop('stale_feedback')
        if not exclusive:return self.stop('control_conflict')
        if not tucked:return self.stop('arm_not_tucked')
        if not collision_clear:return self.stop('clearance_not_confirmed')
        if math.dist(pose[:2],self.last_pose[:2])>.01:return self.stop('pose_jump')
        self.last_time=now;self.last_pose=tuple(pose)
        yaw=math.atan2(math.sin(pose[2]-self.start[2]),math.cos(pose[2]-self.start[2]))
        if abs(yaw)>.03:return self.stop('yaw_drift')
        dx=pose[0]-self.start[0];dy=pose[1]-self.start[1];c=math.cos(self.start[2]);s=math.sin(self.start[2])
        x=c*dx+s*dy;y=-s*dx+c*dy
        length=math.hypot(*self.delta);ux,uy=(v/length for v in self.delta)
        along=x*ux+y*uy;cross=-x*uy+y*ux
        if abs(cross)>.005 or along<-.005:return self.stop('path_drift')
        if along>=length-.002-1e-12:return self.stop('step_complete')
        speed=min(.01, max(0.,(length-along)*1.5))
        # Rotate fixed start-frame travel direction into the current base frame.
        cy=math.cos(yaw);sy=math.sin(yaw)
        return dict(vx=speed*(cy*ux+sy*uy),vy=speed*(-sy*ux+cy*uy),wz=0.,state='moving')
