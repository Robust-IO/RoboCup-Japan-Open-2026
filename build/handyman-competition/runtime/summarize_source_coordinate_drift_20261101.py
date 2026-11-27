import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');sys.path[:0]=[str(b),'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1']
from hand_camera_ray import rotation
from horizontal_can_grasp_plan import forward
from inverse_table_place_plan import FLEX,WRIST
from table_approach_reachability import base_path
audit=json.loads((b/'cigarette-history-depth-and-gates-audit-20261101.json').read_text());table=audit['table'];p,r=forward([.25,FLEX,0,WRIST,0,.65]);contact=p+r@np.array([.0000000685,.00005,.0621258659]);rows=[]
for run in ('cigarette-grasp-20261101-01','cigarette-grasp-20261101-07','cigarette-grasp-20261101-09','cigarette-grasp-20261101-12'):
 root=b/run;cap=root/'captures/source';s=json.loads((cap/'snapshot.json').read_text());m=s['tracking_transforms']['map_from_base_at_depth'];o=s['tracking_transforms']['odom_from_base_at_depth'];rm=rotation(m['quaternion']);ro=rotation(o['quaternion']);rmo=rm@ro.T;tmo=np.asarray(m['translation'])-rmo@o['translation']
 row=next(x for x in audit['runs'] if x['run']==run)['frames'][0];target=row.get('targets',[{}])[0];c=np.asarray(target.get('center_base',[float('nan')]*3));delta=np.r_[c[:2]-contact[:2],0]
 rows.append(dict(run=run,map_from_odom_translation=tmo.tolist(),map_from_odom_yaw=float(Rotation.from_matrix(rmo).as_euler('xyz')[2]),
  rgb_intrinsics=s['rgb_k'],base_from_rgb=s['base_from_rgb'],joint_head=[s['joints'][k] for k in ('head_pan_joint','head_tilt_joint')],
  observed_front_map_m=row.get('observed_front',{}).get('observed_front_map_m'),nominal_reachability=base_path(row['pose'],delta,table),
  target_map=target.get('center_map'),target_in_odom=(ro@c+o['translation']).tolist()))
(b/'cigarette-coordinate-drift-summary-20261101.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows))
