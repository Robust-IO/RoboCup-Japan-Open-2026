"""Conservative TF-window motion estimate, not measured odometry or ground truth.

Consumes odom -> base_footprint poses. Map pose is independently required for
arrival. Sub-threshold motion and motion between samples cannot be ruled out.
"""
from collections import deque
import math


def angle(a,b):
    return abs(math.atan2(math.sin(a-b),math.cos(a-b)))


class TfMotionGate:
    def __init__(self, max_gap_s=.3):
        if not math.isfinite(max_gap_s) or not 0<max_gap_s<=.5:
            raise ValueError('invalid_tf_gap_limit')
        self.max_gap_s=max_gap_s
        self.samples=deque(maxlen=64)
        self.last_stamp=0
        self.last_now=None

    def reset(self):
        self.samples.clear()

    def update(self,pose,stamp_ns,sensor_now_ns,now,*,parent='odom',child='base_footprint'):
        def result(reason,**extra):
            return dict(source='tf_pose_window_estimate',stationary=False,valid=False,
                        reason=reason,actionable=False,**extra)
        def reject(reason):
            self.reset();return result(reason)
        try:
            if (parent,child)!=('odom','base_footprint'):return reject('wrong_tf_edge')
            x,y,yaw=(pose[k] for k in ('x','y','yaw'))
            if not all(math.isfinite(v) for v in (x,y,yaw,now)):return reject('nonfinite_pose_or_time')
            if not all(type(v) is int for v in (stamp_ns,sensor_now_ns)):return reject('invalid_stamp')
            if stamp_ns<=self.last_stamp or not 0<=sensor_now_ns-stamp_ns<=500_000_000:
                return reject('stale_or_reversed_tf')
            gap=self.last_now is not None and (now<=self.last_now or now-self.last_now>self.max_gap_s or stamp_ns-self.last_stamp>int(self.max_gap_s*1e9))
            self.last_stamp=stamp_ns;self.last_now=now
            if gap:self.reset()
            self.samples.append((stamp_ns,now,x,y,yaw))
            while (len(self.samples)>6 and stamp_ns-self.samples[1][0]>=1_000_000_000
                   and now-self.samples[1][1]+1e-9>=1.):
                self.samples.popleft()
            if len(self.samples)<6 or stamp_ns-self.samples[0][0]<1_000_000_000 or now-self.samples[0][1]+1e-9<1.:
                return result('collecting_fresh_tf_window',count=len(self.samples))
            points=list(self.samples);duration=(stamp_ns-points[0][0])/1e9
            linear=sum(math.hypot(b[2]-a[2],b[3]-a[3]) for a,b in zip(points,points[1:]))/duration
            angular=sum(angle(b[4],a[4]) for a,b in zip(points,points[1:]))/duration
            spread=max(math.hypot(a[2]-b[2],a[3]-b[3]) for a in points for b in points)
            yaw_spread=max(angle(a[4],b[4]) for a in points for b in points)
            stationary=linear<=.02 and angular<=.03 and spread<=.004 and yaw_spread<=.005
            return dict(source='tf_pose_window_estimate',valid=True,stationary=stationary,
                reason='within_stationary_thresholds' if stationary else 'motion_detected',
                linear_speed_estimate=linear,angular_speed_estimate=angular,
                position_spread_m=spread,yaw_spread_rad=yaw_spread,stamp_ns=stamp_ns,
                count=len(points),window_s=duration,actionable=False)
        except (KeyError,TypeError,ValueError,OverflowError):return reject('malformed_tf')


class TfSearchArrival:
    """Offline adapter. Caller must supply map pose at exactly the TF stamp."""
    def __init__(self,arrival, max_gap_s=.3):
        self.arrival=arrival
        self.motion=TfMotionGate(max_gap_s)

    def sample(self,odom_pose,map_pose,stamp_ns,map_stamp_ns,sensor_now_ns,now):
        if self.arrival.scan.phase!='navigate':
            return dict(self.arrival.scan.status(),arrival_reason='arrival_adapter_inactive')
        estimate=self.motion.update(odom_pose,stamp_ns,sensor_now_ns,now)
        self.arrival.scan.tick(now)
        if map_stamp_ns!=stamp_ns or map_pose is None or not estimate.get('stationary'):
            self.arrival.reset()
            return dict(self.arrival.scan.status(),motion=estimate,
                        arrival_reason='map_pose_missing_or_mismatched' if map_stamp_ns!=stamp_ns or map_pose is None else 'tf_motion_not_ready')
        state=self.arrival.sample(pose=map_pose,linear_speed=estimate['linear_speed_estimate'],
            angular_speed=estimate['angular_speed_estimate'],stamp_ns=stamp_ns,
            sensor_now_ns=sensor_now_ns,now=now)
        return dict(state,motion=estimate)


class TfSearchObserver:
    """Arrival plus observation watchdog; never emits robot/protocol commands."""
    def __init__(self, arrival, now, *, recover_transient_tf=False, max_gap_s=.3):
        self.arrival = arrival
        self.adapter = TfSearchArrival(arrival,max_gap_s)
        self.max_gap_s=max_gap_s
        self.last_valid = now
        self.recover_transient_tf = recover_transient_tf
        self.recovery_used = False
        self.recovery_deadline = None

    def invalidate(self, reason):
        self.adapter.motion.reset()
        self.arrival.reset()
        scan = self.arrival.scan
        if scan.phase == 'observe':
            scan.phase = 'incomplete'
            scan.position = None
            scan.failures.append(reason)
        return dict(scan.status(), observer_reason=reason)

    def tick(self, now):
        scan = self.arrival.scan
        if scan.phase not in ('navigate', 'observe'):
            return None
        scan.tick(now)
        if scan.phase not in ('navigate', 'observe'):
            return None
        if self.recovery_deadline is not None:
            if now >= self.recovery_deadline:
                scan.phase = 'incomplete'
                scan.position = None
                scan.failures.append('tf_stream_timeout')
                return self.invalidate('tf_stream_timeout')
            if now-self.last_valid > self.max_gap_s:
                self.adapter.motion.reset()
                self.arrival.reset()
            return None
        if now-self.last_valid > self.max_gap_s:
            if scan.phase == 'observe' and self.recover_transient_tf and not self.recovery_used:
                self.recovery_used = True
                self.recovery_deadline = min(scan.deadline, now+4.)
                self.adapter.motion.reset()
                self.arrival.reset()
                self.arrival.adapter = None
                # Retain the same successful goal, but require an entirely new
                # pose/settling window and fresh images. Never dispatch motion.
                scan.phase = 'navigate'
                scan.position = None
                scan.view_start = None
                scan.last_stamp = 0
                scan.negative = 0
                return dict(scan.status(), observer_reason='tf_reacquiring',
                            recovery_deadline=self.recovery_deadline)
            return self.invalidate('tf_stream_timeout')
        return None

    def sample(self, odom_pose, map_pose, stamp_ns, sensor_now_ns, now):
        scan = self.arrival.scan
        self.tick(now)
        if scan.phase not in ('navigate','observe'):
            return scan.status()
        if scan.phase == 'navigate':
            result = self.adapter.sample(odom_pose,map_pose,stamp_ns,stamp_ns,sensor_now_ns,now)
            if self.adapter.motion.last_stamp == stamp_ns and self.adapter.motion.last_now == now:
                self.last_valid = now
            if result.get('arrival_reason') == 'arrived_and_settled':
                self.recovery_deadline = None
            return result
        estimate = self.adapter.motion.update(odom_pose,stamp_ns,sensor_now_ns,now)
        if not estimate.get('stationary'):
            return dict(self.invalidate('motion_or_invalid_tf_during_observation'),motion=estimate)
        try:
            x,y,yaw = (map_pose[k] for k in ('x','y','yaw'))
            ax,ay,ayaw = self.arrival.anchor
            if not all(math.isfinite(v) for v in (x,y,yaw)):
                raise ValueError('nonfinite_map_pose')
            if math.hypot(x-ax,y-ay) > .02 or angle(yaw,ayaw) > .03:
                return self.invalidate('map_pose_changed_during_observation')
        except (TypeError,KeyError,ValueError):
            return self.invalidate('invalid_map_pose')
        self.last_valid = now
        return dict(scan.status(), motion=estimate)
