"""Stop -> capture -> compute -> validate -> act. Pure state bookkeeping."""
import math
import numpy as np

def state_difference(reference,current,policy):
    a,b=np.asarray(reference['joints'],float),np.asarray(current['joints'],float)
    ha,hb=np.asarray(reference['head'],float),np.asarray(current['head'],float)
    pa,pb=reference['poses']['odom'],current['poses']['odom']
    if a.shape!=(6,) or b.shape!=(6,) or ha.shape!=(2,) or hb.shape!=(2,) or not np.isfinite([*a,*b,*ha,*hb,*pa,*pb]).all():raise ValueError('finite_observation_state_required')
    translation=math.dist(pa[:2],pb[:2]);yaw=abs(math.atan2(math.sin(pa[2]-pb[2]),math.cos(pa[2]-pb[2])))
    changed=(translation>policy['stop_base_translation_m'] or yaw>policy['stop_base_yaw_rad']
        or abs(a[0]-b[0])>policy['stop_arm_lift_m'] or np.max(abs(a[1:]-b[1:]))>policy['stop_arm_angle_rad']
        or np.max(abs(ha-hb))>policy['stop_head_angle_rad'])
    return dict(changed=bool(changed),base_translation_m=translation,base_yaw_rad=yaw,
        lift_difference_m=float(abs(a[0]-b[0])),joint_angle_difference_rad=float(np.max(abs(a[1:]-b[1:]))),
        head_angle_difference_rad=float(np.max(abs(ha-hb))))

class ObservationCycle:
    def __init__(self,policy):
        self.policy=policy;self.generation=0;self.pending_until=0.;self.anchor=None;self.latest_stamp=None;self.samples=0;self.stopped=False;self.stop_stamp_ns=None;self.capture_not_before_ns=0
    def invalidate(self,*,now,duration=0.):
        self.generation+=1;self.pending_until=max(self.pending_until,now+duration)
        self.anchor=None;self.latest_stamp=None;self.samples=0;self.stopped=False;self.stop_stamp_ns=None;self.capture_not_before_ns=0
    def invalidate_map(self,*,capture_after_ns):
        """Map recalibration invalidates coordinates, not physical stop proof."""
        self.generation+=1
        self.capture_not_before_ns=max(self.capture_not_before_ns,capture_after_ns)
    def observe(self,state,*,now):
        if now<self.pending_until:return False
        if self.anchor is None or state_difference(self.anchor,state,self.policy)['changed']:
            self.anchor=state;self.latest_stamp=state['stamp_ns'];self.samples=1;self.stopped=False;self.stop_stamp_ns=None;return False
        if self.stopped:return True
        if state['stamp_ns']<=self.latest_stamp:return False
        self.latest_stamp=state['stamp_ns'];self.samples+=1
        if self.samples>=2 and state['stamp_ns']-self.anchor['stamp_ns']>=int(self.policy['stop_window_s']*1e9):
            self.stopped=True;self.stop_stamp_ns=state['stamp_ns'];return True
        return False
    def capture_marker(self,snapshot):
        if not self.stopped:raise ValueError('robot_must_stop_before_capture')
        from observation_profiles import stamps as required_stamps
        stamps=required_stamps(snapshot)
        cutoff=max(self.stop_stamp_ns,self.capture_not_before_ns)
        if min(stamps)<cutoff:raise ValueError('new_image_after_stop_required')
        return dict(policy='stop_observe_act_v1',generation=self.generation,
            stop_verified_stamp_ns=self.stop_stamp_ns,capture_not_before_ns=cutoff,captured_after_stop=True)
