"""Calibrated HSR kinematics shared by all object identities."""
import math
import numpy as np

def ry(a):
 c,s=math.cos(a),math.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
 c,s=math.cos(a),math.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def forward(j):
 lift,flex,arm_roll,wrist,roll,_=j
 if abs(arm_roll)>1e-6:raise ValueError('uncalibrated_arm_roll')
 rot=ry(-flex-wrist)@rz(roll)
 palm=np.array([.1410000026,.0780000016,.3392546773+lift])+ry(-flex)@np.array([.0051020361,0,.3492376804])+rot@np.array([.012,0,.1405])
 return palm,rot@rz(math.pi)
def posture(strategy,angle=0.,lift=0.,opening=.65):
 if strategy=='side':return [lift,-1.7,0.,.1292036732051034,angle,opening]
 if strategy=='overhead':return [lift,-math.pi/2,0.,-math.pi/2,angle,opening]
 raise ValueError('unsupported_strategy')
CONTACT=np.array([0.,.00005,.062125866])
