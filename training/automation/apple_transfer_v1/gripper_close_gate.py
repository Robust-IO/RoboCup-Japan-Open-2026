"""Closure evidence only: a stopped gripper is NOT proof of grasp success."""
import math


class CloseGate:
    def __init__(self, initial_opening, duration=6.):
        if not math.isfinite(initial_opening) or not .05<initial_opening<=1.24:
            raise ValueError('invalid_initial_opening')
        if not math.isfinite(duration) or duration<=0:raise ValueError('invalid_duration')
        self.initial=initial_opening;self.duration=duration;self.rows=[];self.last_stamp=None

    def reset_window(self):
        self.rows=[]

    def observe(self, elapsed, stamp_ns, opening, arm_error):
        if not all(math.isfinite(v) for v in (elapsed,stamp_ns,opening,arm_error)):
            raise ValueError('invalid_closure_feedback')
        if elapsed<0 or arm_error<0 or not -.005<=opening<=1.24:
            raise ValueError('invalid_closure_feedback')
        if self.last_stamp is not None and stamp_ns<self.last_stamp:
            raise ValueError('closure_clock_reversed')
        if stamp_ns==self.last_stamp:return None
        self.last_stamp=stamp_ns
        if arm_error>.005:raise ValueError('arm_moved_during_close')
        if elapsed<self.duration:
            self.reset_window();return None
        if self.rows and (elapsed-self.rows[-1][0]>.4 or (stamp_ns-self.rows[-1][1])/1e9>.4):
            self.reset_window()
        self.rows.append((elapsed,stamp_ns,opening))
        # Start a new stability window on movement; don't hide motion by averaging.
        if max(r[2] for r in self.rows)-min(r[2] for r in self.rows)>.003:
            self.rows=self.rows[-1:]
        first=self.rows[0]
        if len(self.rows)<4 or elapsed-first[0]<.6 or (stamp_ns-first[1])/1e9<.6:return None
        result=dict(observed_opening_rad=opening,closing_travel_rad=self.initial-opening,
                    stable_samples=len(self.rows),stable_duration_s=elapsed-first[0],
                    grasp_success_verified=False,requires_visual_lift_verification=True)
        if abs(opening)<=.005:result['state']='closed_without_contact_evidence'
        elif opening>.02 and self.initial-opening>=.03:result['state']='contact_candidate'
        else:result['state']='closure_inconclusive'
        return result
