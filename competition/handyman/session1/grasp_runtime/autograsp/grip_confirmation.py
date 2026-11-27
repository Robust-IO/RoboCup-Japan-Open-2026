"""Joint-feedback grasp acceptance; does not claim visual object motion."""
import math

def accepted(e):
    common=e.get('verified') is True and e.get('grip_retained') is True and e.get('observations',0)>=3
    minimum=e.get('minimum_lift_m',.05)
    valid_minimum=isinstance(minimum,(int,float)) and math.isfinite(minimum) and .02<=minimum<=.05
    return common and (e.get('object_followed') is True or
        (e.get('basis')=='lift_and_reclose_joint_feedback' and valid_minimum and e.get('lift_distance_m',0)+1e-6>=minimum and e.get('reclose_blocked') is True))

class RecloseEvidence:
    def __init__(self):self.rows=[];self.last_stamp=0
    def observe(self,stamp,opening):
        if stamp<=self.last_stamp:return None
        self.last_stamp=stamp
        if not math.isfinite(opening):raise ValueError('invalid_gripper_feedback')
        if opening<=.01:return False
        self.rows.append((stamp,opening));self.rows=self.rows[-30:]
        recent=[r for r in self.rows if stamp-r[0]<=400_000_000]
        if len(recent)>=3 and stamp-recent[0][0]>=200_000_000 and max(r[1] for r in recent)-min(r[1] for r in recent)<=.01:return True
        return None
