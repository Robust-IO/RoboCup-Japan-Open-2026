"""Geometry for the explicit simulated horizontal-can top-down trial.

Link offsets come from measured HSR TF. Joint limits come from the supplied
HSRCommon.cs. Existing upright-can/apple/pepper profiles are not modified.
"""
import math
import numpy as np

SHOULDER=np.array([.1410000026,.0780000016,.3392546773])
UPPER=np.array([.0051020361,0.,.3492376804])
PALM=np.array([.012,0.,.1405])
CONTACT_LOCAL=np.array([0.,.00005,.0794860814])
LIMITS=((0.,.69),(-2.617,0.),(-1.919,3.665),(-1.919,1.221),(-1.919,3.665),(0.,.65))

def ry(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rz(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def validate_goal(goal):
    if len(goal)!=6 or not np.isfinite(goal).all() or any(not lo-1e-7<=v<=hi+1e-7 for v,(lo,hi) in zip(goal,LIMITS)):
        raise ValueError('horizontal_trial_goal_outside_unity_joint_limits')
    if abs(goal[2])>1e-7:
        raise ValueError('horizontal_trial_arm_roll_not_calibrated')
    return list(map(float,goal))

def forward(goal):
    lift,flex,_,wrist,roll,_=validate_goal(goal)
    shoulder=SHOULDER+np.array([0.,0.,lift])
    wrist_xyz=shoulder+ry(-flex)@UPPER
    rot=ry(-flex-wrist)@rz(roll)
    palm=wrist_xyz+rot@PALM
    return palm,rot@rz(math.pi)

def contact(palm):
    from hand_camera_ray import rotation
    return np.asarray(palm['translation'])+rotation(palm['quaternion'])@CONTACT_LOCAL

def overhead_plan(measurement,clearance=.15):
    center=np.asarray(measurement['center_base_m'],float)
    axis=np.asarray(measurement['axis_base'],float)
    if center.shape!=(3,) or axis.shape!=(3,) or not np.isfinite([center,axis]).all():
        raise ValueError('measured_horizontal_can_required')
    if abs(axis[2])>.15 or not .01<=measurement['radius_m']<=.06:
        raise ValueError('horizontal_can_shape_required')
    yaw=math.atan2(axis[1],axis[0])
    roll=(math.pi-yaw+math.pi/2)%math.pi-math.pi/2
    base=[0.,-math.pi/2,0.,-math.pi/2,roll,.65]
    p,r=forward(base)
    contact_at_zero=p+r@CONTACT_LOCAL
    lift=float(center[2]+clearance-contact_at_zero[2])
    goal=validate_goal([lift,*base[1:]])
    p,r=forward(goal)
    return dict(goal=goal,grasp_lift_m=lift-clearance,
        predicted_palm_m=p.tolist(),predicted_contact_m=(p+r@CONTACT_LOCAL).tolist(),
        approach_clearance_m=clearance,grasp_from_above=True,motion_authorized=False)
