"""Head-only geometric diagnosis. No base, arm, or gripper commands."""
import json,sys,uuid
from pathlib import Path
import numpy as np
import rclpy
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
import penguin_geometry as g
from penguin_runtime import PenguinRuntime
from canned_grasp_geometry import transform_point
from apple_transfer_automation import RobotLock
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'penguin-head-pan-audit-20261025-01'
previous=json.loads((base/'penguin-native-transfer-20261025-02/result.json').read_text())
last=previous['source_observations'][-1];source=json.loads((Path(last['capture'])/'snapshot.json').read_text())
ref=last['measurement'];b=transform_point(source,ref['center_m']);h=transform_point(source,ref['cap_center_m'])
result=dict(base_arm_gripper_commands_sent=False,rows=[],diagnostic_only=True)
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init()
 runtime=PenguinRuntime(root,'penguin-pan-audit-'+uuid.uuid4().hex[:8],lambda _:None)
 try:
  first=runtime.sample();initial_pan=first['snapshot']['joints']['head_pan_joint']
  for index,pan in enumerate((initial_pan,.471148691454172,initial_pan)):
   if index:runtime.face_source([pan,-.5])
   sample=runtime.sample();s=sample['snapshot'];eb=transform_point(s,b,inverse=True);eh=transform_point(s,h,inverse=True)
   fits=[];original=g.fit_sphere
   def traced(points,limits):
    c,r,proof=original(points,limits);fits.append(dict(center=c.tolist(),radius=r,proof=proof));return c,r,proof
   g.fit_sphere=traced
   try:measurement=g.measure_near(sample['rgb'],sample['depth'],s,eb,eh,ref);reason='original_association_passed'
   except ValueError as exc:
    reason=str(exc);measurement=None
   finally:g.fit_sphere=original
   # Refit around actual observed body solely to diagnose a possible rigid shift;
   # this helper never supplies object geometry to any motion command.
   if measurement is None and fits:
    shifted=np.asarray(fits[0]['center'])-eb
    try:measurement=g.measure_near(sample['rgb'],sample['depth'],s,eb+shifted,eh+shifted,ref)
    except ValueError as exc:reason+=';diagnostic_refit:'+str(exc)
   capture=runtime.save(sample,'view-'+str(index),measurement)
   row=dict(pan=pan,original_association=reason,capture=str(capture),fits=fits,
            base_pose=runtime.state()['poses'],measurement=measurement)
   if measurement:
    row['body_odom_m']=transform_point(s,measurement['center_m']).tolist()
    row['head_odom_m']=transform_point(s,measurement['cap_center_m']).tolist()
   result['rows'].append(row)
 except Exception as exc:result['reason']=str(exc)
 finally:
  runtime.shutdown();rclpy.shutdown()
  (root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
  print(json.dumps(result),flush=True)
