import sys,json
from pathlib import Path
import numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1']
from local_handle_geometry import select_pinch,register_surface
from thin_box_clearance import ThinBoxClearance
from horizontal_can_grasp_plan import forward
from table_transfer_core import FLEX,WRIST
from hand_camera_ray import rotation
p=b/'round-placement-recheck-20261107-171522';pts=np.load(p/'controller-points.npy');bc=np.array(json.loads((p/'controller-plane.json').read_text())['plane_base']);s=json.loads((p/'captures/controller-source/snapshot.json').read_text());depth=np.load(p/'captures/controller-source/depth_m.npy');_,flat=forward([.25,FLEX,0,WRIST,0,.65]);c,cs=select_pinch(pts,bc,ThinBoxClearance().envelope,flat);tf=s['tracking_transforms']['odom_from_base_at_depth'];r=rotation(tf['quaternion']);t=np.asarray(tf['translation']);front=np.quantile(pts[:,0],.02);template=pts[(pts[:,0]>front+.055)&(pts[:,2]>pts[:,:2]@bc[:2]+bc[2]+.022)]@r.T+t;g=r@c['center_m']+t
m,shift=register_surface(depth,s,template,g);print(json.dumps(dict(chosen=c,registration=m,shift=shift.tolist())))
assert np.linalg.norm(shift)<.003
m,shift=register_surface(depth,s,template+[.01,0,0],g+[.01,0,0]);print(json.dumps(dict(perturbed_shift=shift.tolist(),measurement=m)))
assert np.linalg.norm(shift-[-.01,0,0])<.005
