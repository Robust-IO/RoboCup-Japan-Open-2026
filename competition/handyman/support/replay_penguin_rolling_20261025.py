import json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from penguin_geometry import measure_near
from canned_grasp_geometry import transform_point
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'penguin-native-transfer-20261025-05'
r=json.loads((root/'result.json').read_text());last=r['source_observations'][-1]
s=json.loads((Path(last['capture'])/'snapshot.json').read_text());model=last['measurement']
b=transform_point(s,model['center_m']);h=transform_point(s,model['cap_center_m']);rows=[]
for p in sorted((root/'grasp/captures').iterdir()):
 if not p.name[:3].isdigit():continue
 s=json.loads((p/'snapshot.json').read_text());rgb=cv2.cvtColor(cv2.imread(str(p/'rgb.png')),cv2.COLOR_BGR2RGB);d=np.load(p/'depth_m.npy')
 try:
  m=measure_near(rgb,d,s,transform_point(s,b,inverse=True),transform_point(s,h,inverse=True),model)
  nb=transform_point(s,m['center_m']);nh=transform_point(s,m['cap_center_m'])
  rows.append(dict(capture=p.name,status='passed',body_delta_m=float(np.linalg.norm(nb-b)),head_delta_m=float(np.linalg.norm(nh-h))))
  b,h=nb,nh
 except ValueError as ex:rows.append(dict(capture=p.name,status='rejected',reason=str(ex)))
result=dict(robot_commands_sent=False,rows=rows,all_passed=all(r['status']=='passed' for r in rows))
with (base/'penguin-rolling-reference-replay-20261025.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
