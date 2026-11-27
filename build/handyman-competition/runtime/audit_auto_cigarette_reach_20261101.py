"""Measure current recorded target and candidate endpoints, no ROS or motion."""
import json,sys
from pathlib import Path
import cv2,numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');sys.path[:0]=[str(b),'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1']
from flat_box_geometry import estimate
from horizontal_can_grasp_plan import forward
from inverse_table_place_plan import FLEX,WRIST
from apple_transfer_auto_plan import map_pose,map_point
from table_approach_reachability import base_path
from thin_box_clearance import ThinBoxClearance
from hand_camera_ray import rotation
from scipy.spatial.transform import Rotation
root=b/'cigarette-grasp-20261101-12/captures/source';s=json.loads((root/'snapshot.json').read_text());rgb=cv2.cvtColor(cv2.imread(str(root/'rgb.png')),cv2.COLOR_BGR2RGB);d=np.load(root/'depth_m.npy')
cfg=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==cfg['source_table_id']);safety=ThinBoxClearance()
p,r=forward([.25,FLEX,0,WRIST,0,.65]);rows=[]
for detection in json.loads((root/'detections.json').read_text())['detections']:
 if detection['name']!='cigarette':continue
 m=estimate(rgb,d,s,detection);c=np.asarray(m['center_m']);contact=p+r@np.array([.0000000685,.00005,.0621258659]);delta=c[:2]-contact[:2]
 checks=[]
 for shift in (0.,-.01,-.02,-.03,-.04,-.05,-.06):
  move=np.r_[delta+[shift,0.],0.];targetp=p+np.r_[move[:2],c[2]-p[2]]
  checks.append(dict(shift=shift,delta=move.tolist(),reach=base_path(map_pose(s),move,table),
    closure=safety.closure_region(dict(translation=targetp,quaternion=Rotation.from_matrix(r).as_quat().tolist()),c,m['axis_base'],m)))
 rows.append(dict(detection=detection,model=m,map_center=map_point(s,c).tolist(),nominal_contact=contact.tolist(),endpoint_checks=checks))
report=dict(robot_commands_sent=False,pose=map_pose(s),table=table,targets=rows);(b/'cigarette-auto-source12-reach-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
