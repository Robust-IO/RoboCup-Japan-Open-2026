"""Restore the measured overhead grasp orientation for support-preserving release."""
import numpy as np
from grasp_runtime.autograsp.kinematics import forward

class MeasuredReleaseCore:
    def __init__(self,base,joints):
        self.base=base;self.joints=list(joints)
        zero=list(joints);zero[0]=0.;self.offset=float(forward(zero)[0][2])
        self.rotation=forward(zero)[1]
    def predict(self,height,center_minus_palm):
        goal=list(self.joints);goal[0]=height-self.offset
        palm,_=forward(goal)
        return np.asarray(palm)+np.asarray(center_minus_palm)
    def prepare_release(self,runtime,height,opening,*,already_extended=False):
        lift=float(height)-self.offset
        if not 0<=lift<=.69:raise ValueError('measured_release_height_outside_lift_range')
        state=runtime.state();goal=list(state['joints'])
        # The existing standoff measurement runs before extending the arm.
        goal[0]=max(goal[0],min(.69,lift+.06))
        runtime.move_joints(goal,'raise_for_measured_release_posture')
        goal=list(runtime.state()['joints']);goal[1:5]=self.joints[1:5]
        runtime.move_joints(goal,'restore_measured_release_orientation')
        goal=list(runtime.state()['joints']);goal[0]=lift
        runtime.move_joints(goal,'measured_release_height')
    def release_open(self,*a,**kw):return self.base.release_open(*a,**kw)
    def raise_after_release(self,*a,**kw):return self.base.raise_after_release(*a,**kw)
    def fold_after_withdrawal(self,*a,**kw):return self.base.fold_after_withdrawal(*a,**kw)
