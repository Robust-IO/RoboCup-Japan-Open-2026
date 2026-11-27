import json,sys,importlib.util
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
spec=importlib.util.spec_from_file_location('penguin_closed_candidate','/__HANDYMAN_PACKAGE__/runtime/penguin-closed-geometry-candidate-20261025.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
from canned_grasp_geometry import transform_point
base=Path('/__HANDYMAN_PACKAGE__/runtime');prior=base/'penguin-native-transfer-20261025-10';p=json.loads((prior/'result.json').read_text())
before_path=Path(p['close']['before_capture']);bs=json.loads((before_path/'snapshot.json').read_text());before=json.loads((before_path/'object-measurement.json').read_text());model=p['source_observations'][-1]['measurement']
b=transform_point(bs,before['center_m']);h=transform_point(bs,before['cap_center_m']);palm=transform_point(bs,bs['palm_base_at_depth']);rows=[]
folders=[prior/'grasp/captures/014-near-rejected',prior/'grasp/captures/018-near-rejected']+sorted((base/'penguin-current-head-lift-test-20261025-01/captures').iterdir())
for folder in folders:
 s=json.loads((folder/'snapshot.json').read_text());rgb=cv2.cvtColor(cv2.imread(str(folder/'rgb.png')),cv2.COLOR_BGR2RGB);d=np.load(folder/'depth_m.npy')
 move=transform_point(s,s['palm_base_at_depth'])-palm
 try:
  m=g.measure_near(rgb,d,s,transform_point(s,b+move,inverse=True),transform_point(s,h+move,inverse=True),model)
  rows.append(dict(capture=str(folder),status='passed',head_fit=m['cap_fit'],head_delta_odom_m=(transform_point(s,m['cap_center_m'])-h).tolist(),palm_delta_odom_m=move.tolist(),measurement=m))
 except ValueError as exc:rows.append(dict(capture=str(folder),status='rejected',reason=str(exc)))
result=dict(robot_commands_sent=False,rows=rows,all_passed=all(r['status']=='passed' for r in rows))
with (base/'penguin-closed-head-depth-replay-20261025.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
