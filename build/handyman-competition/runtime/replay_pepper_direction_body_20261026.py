import importlib.util,json,sys,time
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
from canned_grasp_geometry import transform_point
from pepper_geometry import measure_near
spec=importlib.util.spec_from_file_location('candidate','/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pepper_visual_motion_candidate_20261026.py');v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v)
specg=importlib.util.spec_from_file_location('candidate_geometry','/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pepper_geometry_candidate_20261026.py');g=importlib.util.module_from_spec(specg);specg.loader.exec_module(g);measure_near=g.measure_near
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');out=[]
for name in ('pepper-frozen-cross-object-transfer-20261026-01','pepper-compact-realign-full-transfer-20261026-01','pepper-visual-motion-full-transfer-20261026-01','pepper-motion-direction-full-transfer-20261026-01'):
 root=base/name;model=json.loads((root/'grasp/source-reference.json').read_text());anchor=None;body_odom=None;cap_odom=None
 folders=sorted((root/'grasp/captures').glob('*-near*'))
 for p in folders:
  sample=dict(snapshot=json.loads((p/'snapshot.json').read_text()),depth=np.load(p/'depth_m.npy'),rgb=cv2.imread(str(p/'rgb.png'))[:,:,::-1].copy())
  s=sample['snapshot'];row=dict(run=name,capture=p.name)
  if anchor is None:
   if not (p/'object-measurement.json').exists():continue
   m=json.loads((p/'object-measurement.json').read_text());anchor=v.NearAnchor(sample,m)
   body_odom=transform_point(s,m['center_m']);cap_odom=transform_point(s,m['cap_center_m']);continue
  start=time.monotonic()
  try:
   eb=transform_point(s,body_odom,inverse=True);ec=transform_point(s,cap_odom,inverse=True)
   predicted=anchor.predict(sample,eb,ec)
   if predicted is not None:eb,ec,evidence=predicted;row['static_motion']=evidence
   m=measure_near(sample['rgb'],sample['depth'],s,eb,ec,model)
   row.update(valid=True,measured_center=m['center_m'],measured_head=m['cap_center_m'],body_fit=m['body_fit'])
   anchor=v.NearAnchor(sample,m);body_odom=transform_point(s,m['center_m']);cap_odom=transform_point(s,m['cap_center_m'])
  except ValueError as e:row.update(valid=False,reason=str(e))
  row['processing_s']=time.monotonic()-start;out.append(row)
report=dict(robot_commands_sent=False,replay_only=True,rows=out)
p=base/'pepper-visual-direction-body-offline-replay-20261026.json';p.write_text(json.dumps(report,indent=2));print(json.dumps(report))
