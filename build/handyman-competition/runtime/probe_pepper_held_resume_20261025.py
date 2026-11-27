import sys,json,time,uuid
from pathlib import Path
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
import numpy as np
import rclpy
from pepper_runtime import PepperRuntime
from pepper_hold import current_depth,rgb_continuation,expected_centers
from canned_carry_geometry import current_images_and_feedback
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pepper-current-lifted-hold-probe-20261025-01');root.mkdir(exist_ok=False)
source=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pepper-resume-final-round-20261025-01/legs/00-initial-reset');held=json.loads((source/'held-context.json').read_text());result=dict(robot_commands_sent=False,source_run=str(source),rows=[])
rclpy.init();rt=PepperRuntime(root,'pepper-hold-readonly-'+uuid.uuid4().hex[:8],lambda _:None)
try:
 for i in range(3):
  sample=rt.sample();s=sample['snapshot'];state=rt.state();rt.ownership();palm=rt.palm_pose(s);b,c=expected_centers(palm,held)
  row=dict(current_state=state,depth_range_m=[float(np.nanmin(sample['depth'])),float(np.nanmax(sample['depth']))],predicted_roi_only=dict(body_base=b.tolist(),cap_base=c.tolist()))
  try:row['depth_evidence']=current_depth(sample['rgb'],sample['depth'],s,palm,held)
  except Exception as exc:row['depth_rejection']=str(exc)
  try:row['rgb_evidence']=rgb_continuation(sample['rgb'],sample['depth'],s,palm,held)
  except Exception as exc:row['rgb_rejection']=str(exc)
  try:row['freshness']=current_images_and_feedback(s,rt.state(),rt.node.get_clock().now().nanoseconds,rt.epoch,joint_max_age_ns=600_000_000)
  except Exception as exc:row['freshness_rejection']=str(exc)
  row['capture']=str(rt.save(sample,'held-'+str(i)));result['rows'].append(row)
except Exception as exc:result['reason']=str(exc)
finally:
 rt.shutdown();rclpy.shutdown();(root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False));print(json.dumps({k:v for k,v in result.items() if k!='rows'}));print(json.dumps([dict(capture=r['capture'],depth_rejection=r.get('depth_rejection'),rgb_rejection=r.get('rgb_rejection'),has_depth='depth_evidence' in r,has_rgb='rgb_evidence' in r,freshness_rejection=r.get('freshness_rejection'),state=r['current_state']) for r in result['rows']]))
