import json,time,sys,hashlib,uuid
from pathlib import Path
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
import numpy as np
import rclpy
from pepper_runtime import PepperRuntime
from task_outcome_policy import robot_ready
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'pepper-resume-readonly-20261025-01';root.mkdir(exist_ok=False)
checkpoint=json.loads((base/'ground-pepper-phase5-pause-checkpoint-20261025.json').read_text())
for tree in (Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training'),Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/automation/apple_transfer_v1')):
 for name,want in checkpoint['latest_expected_hashes'].items():
  if hashlib.sha256((tree/name).read_bytes()).hexdigest()!=want:raise ValueError('deployed_source_changed:'+str(tree/name))
rclpy.init();runtime=PepperRuntime(root,'pepper-sync-probe-'+uuid.uuid4().hex[:8],lambda _:None)
rows=[];started=time.monotonic();result=dict(robot_commands_sent=False,three_code_copies_verified=True)
try:
 while time.monotonic()-started<6:
  runtime.pump(.01)
  if not all(k in runtime.received for k in ('rgb','depth','hand','joints')):continue
  now=runtime.node.get_clock().now().nanoseconds
  rows.append({k:(now-runtime.stamp(runtime.received[k]))/1e9 for k in ('rgb','depth','hand','joints')})
 if not rows:raise RuntimeError('no_live_unity_rgbd_and_joint_streams')
 result['age_ranges_s']={k:dict(min=float(np.min([r[k] for r in rows])),median=float(np.median([r[k] for r in rows])),max=float(np.max([r[k] for r in rows]))) for k in ('rgb','depth','hand','joints')}
 sample=runtime.sample();state=runtime.state();runtime.ownership()
 capture=runtime.save(sample,'current-scene')
 names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
 result.update(current_fresh_state=state,empty_tucked=robot_ready(dict(zip(names,state['joints']))),capture=str(capture),source_timestamps_preserved=True)
except Exception as exc:result['reason']=str(exc)
finally:
 runtime.shutdown();rclpy.shutdown();(root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False));print(json.dumps(result),flush=True)
