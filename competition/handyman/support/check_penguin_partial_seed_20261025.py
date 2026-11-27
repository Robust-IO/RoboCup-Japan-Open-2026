import importlib.util,json,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
spec=importlib.util.spec_from_file_location('penguin_seed_candidate','/__HANDYMAN_PACKAGE__/runtime/penguin-seed-geometry-candidate-20261025.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
from can_destination_surface import measure
from canned_grasp_geometry import transform_point
root=Path('/__HANDYMAN_PACKAGE__/runtime/penguin-native-transfer-20261025-07');p=root/'grasp/captures/identity-00'
cfg=json.loads((root/'config.snapshot.json').read_text());source=next(x for x in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if x['id']==cfg['source_table_id'])
rgb=cv2.cvtColor(cv2.imread(str(p/'rgb.png')),cv2.COLOR_BGR2RGB);depth=np.load(p/'depth_m.npy');s=json.loads((p/'snapshot.json').read_text());d=next(x for x in json.loads((p/'raw-detector-result.json').read_text())['detections'] if x['name']=='toy_penguin')
plane=measure(depth,s,source);seed=g.estimate_source(rgb,depth,s,d,measured_support=plane,allow_clipped_seed=True)
b=transform_point(s,seed['center_m']);h=transform_point(s,seed['cap_center_m'])
p=root/'grasp/captures/identity-01';rgb=cv2.cvtColor(cv2.imread(str(p/'rgb.png')),cv2.COLOR_BGR2RGB);depth=np.load(p/'depth_m.npy');s=json.loads((p/'snapshot.json').read_text())
eb=transform_point(s,b,inverse=True);eh=transform_point(s,h,inverse=True)
window,u,v=g.near_window(depth.shape,s,[eb,eh],[seed['observed_horizontal_extent_m']+.015,seed['cap_radius_m']+.015])
proposal=dict(name='toy_penguin',confidence=d['confidence'],box=[float(u.min()),float(v.min()),float(u.max()+1),float(v.max()+1)],identity_source='earlier_high_confidence_seed_plus_new_rgbd')
current=g.estimate_source(rgb,depth,s,proposal,measured_support=measure(depth,s,source))
result=dict(robot_commands_sent=False,seed=seed,current=current,proposal=proposal,
 body_odom_error_m=float(np.linalg.norm(transform_point(s,current['center_m'])-b)),
 head_odom_error_m=float(np.linalg.norm(transform_point(s,current['cap_center_m'])-h)))
out=Path('/__HANDYMAN_PACKAGE__/runtime/penguin-identity-seed-replay-20261025.json')
with out.open('x') as stream:json.dump(result,stream,indent=2)
print(json.dumps(result))
