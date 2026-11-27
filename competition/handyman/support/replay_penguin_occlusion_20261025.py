import json,sys,importlib.util
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
spec=importlib.util.spec_from_file_location('penguin_candidate','/__HANDYMAN_PACKAGE__/runtime/penguin-occluded-geometry-candidate-20261025.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
from canned_grasp_geometry import transform_point
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'penguin-native-transfer-20261025-03'
report=json.loads((root/'result.json').read_text());last=report['source_observations'][-1]
source=json.loads((Path(last['capture'])/'snapshot.json').read_text());ref=last['measurement']
b=transform_point(source,ref['center_m']);h=transform_point(source,ref['cap_center_m']);rows=[]
for name in ('009-near','010-near-rejected','014-near-rejected'):
 p=root/'grasp/captures'/name;s=json.loads((p/'snapshot.json').read_text())
 rgb=cv2.cvtColor(cv2.imread(str(p/'rgb.png')),cv2.COLOR_BGR2RGB);d=np.load(p/'depth_m.npy')
 try:
  m=g.measure_near(rgb,d,s,transform_point(s,b,inverse=True),transform_point(s,h,inverse=True),ref)
  rows.append(dict(capture=name,measurement=m,status='passed'))
 except ValueError as ex:rows.append(dict(capture=name,status='rejected',reason=str(ex)))
result=dict(robot_commands_sent=False,rows=rows,all_passed=all(r['status']=='passed' for r in rows))
with (base/'penguin-occlusion-replay-20261025.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result))
