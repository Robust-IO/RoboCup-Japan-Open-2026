"""Planar finite-difference velocity estimate, NOT independent wheel odometry.

Invalid samples and timeouts have no velocity value. They must never be turned
into zero-speed Odometry messages or evidence that the robot stopped.
"""
from collections import deque
import math


class OdometryDiagnostics:
    """Log-only smoothing and health; never changes the raw twist or proves a stop."""
    def __init__(self):
        self.filtered = None
        self.stamp = None
        self.sequence = 0

    def annotate(self, row):
        if row is None:
            return None
        self.sequence += 1
        result = dict(row)
        if not row['valid']:
            self.filtered = self.stamp = None
            state = ('stopped' if row['reason'] == 'probe_stopped' else
                     'warming' if row['reason'] in ('warming_up', 'gap_rewarming') else 'unavailable')
        else:
            stamp = row['stamp_ns']
            raw = row['twist']
            if self.stamp is None or stamp <= self.stamp:
                self.filtered = dict(raw)
            else:
                alpha = -math.expm1(-(stamp-self.stamp)/1e9/.25)
                self.filtered = {k: self.filtered[k]+alpha*(raw[k]-self.filtered[k]) for k in raw}
            self.stamp = stamp
            result['diagnostic_filtered_twist'] = dict(self.filtered)
            # No deadband: even very slow actual motion must remain visible.
            state = 'available'
        result['health'] = dict(state=state, sequence=self.sequence,
            stop_confirmed=False, navigation_authorized=False,
            consumer_timeout_required=True)
        return result


class TfDerivedOdometry:
    def __init__(self):
        self.samples=deque(maxlen=4)

    def invalidate(self,reason):
        self.samples.clear()
        return dict(valid=False,reason=reason,source='tf_finite_difference',actionable=False)

    def update(self,pose,stamp_ns,sensor_now_ns,now):
        try:
            values=[pose[k] for k in ('x','y','yaw')]
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in values+[now]):
                return self.invalidate('invalid_pose_or_clock')
            if (type(stamp_ns) is not int or type(sensor_now_ns) is not int or stamp_ns<=0 or
                not 0<=sensor_now_ns-stamp_ns<=250_000_000):
                return self.invalidate('stale_future_or_invalid_stamp')
            x,y,yaw=values
            sample=(stamp_ns,now,x,y,yaw)
            if self.samples:
                prev=self.samples[-1];dt=(stamp_ns-prev[0])/1e9;elapsed=now-prev[1]
                if dt<=0 or elapsed<=0:return self.invalidate('nonincreasing_time')
                if dt>.3 or elapsed>.3:
                    self.samples.clear();self.samples.append(sample)
                    return dict(valid=False,reason='gap_rewarming',source='tf_finite_difference',actionable=False)
                if dt<.02:return self.invalidate('interval_too_short')
                dyaw=math.atan2(math.sin(yaw-prev[4]),math.cos(yaw-prev[4]))
                if math.hypot(x-prev[2],y-prev[3])/dt>1.0 or abs(dyaw)/dt>2.0:
                    return self.invalidate('pose_jump_or_excessive_speed')
            self.samples.append(sample)
            duration=(stamp_ns-self.samples[0][0])/1e9
            if len(self.samples)<3 or duration<.2:
                return dict(valid=False,reason='warming_up',source='tf_finite_difference',actionable=False)
            first=self.samples[0]
            vx_world=(x-first[2])/duration;vy_world=(y-first[3])/duration
            angle=sum(math.atan2(math.sin(b[4]-a[4]),math.cos(b[4]-a[4]))
                for a,b in zip(self.samples,list(self.samples)[1:]))
            c,s=math.cos(yaw),math.sin(yaw)
            return dict(valid=True,reason='estimated',source='tf_finite_difference',actionable=False,
                stamp_ns=stamp_ns,frame_id='odom',child_frame_id='base_footprint',
                pose=dict(x=x,y=y,yaw=yaw),window_s=duration,
                twist=dict(vx=c*vx_world+s*vy_world,vy=-s*vx_world+c*vy_world,wz=angle/duration))
        except (KeyError,TypeError,ValueError,OverflowError):return self.invalidate('malformed_sample')

    def tick(self,sensor_now_ns,now):
        if not self.samples:return None
        last=self.samples[-1]
        if not math.isfinite(now) or now<last[1] or now-last[1]>.3:
            return self.invalidate('receive_timeout_or_clock_reset')
        if type(sensor_now_ns) is not int or not 0<=sensor_now_ns-last[0]<=250_000_000:
            return self.invalidate('sensor_clock_stale_or_reset')
        return None
