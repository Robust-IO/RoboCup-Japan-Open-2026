import json,sys,time,uuid
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
import rclpy
from pepper_runtime import PepperRuntime
from apple_transfer_automation import RobotLock
from task_outcome_policy import pose_matches,hand_is_open,robot_ready
b=Path('/__HANDYMAN_PACKAGE__/runtime');root=b/'pepper-surface-empty-arm-restore-20261026-01'
old=json.loads((b/'pepper-motion-direction-full-transfer-20261026-01/result.json').read_text())
if old['close_commands_sent'] or old['release_commands_sent']:raise ValueError('zero_close_and_release_required')
root.mkdir(exist_ok=False);result=dict(status='starting',close_commands_sent=0,release_commands_sent=0,base_commands_sent=False)
with RobotLock():
 rclpy.init();runtime=None
 with (root/'journal.jsonl').open('x') as log:
  def record(e):log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush()
  try:
   runtime=PepperRuntime(root,'empty-restore-'+uuid.uuid4().hex,record)
   s=runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
   if not hand_is_open(state['joints'][5]) or not pose_matches(state['joints'],old['last_fresh_robot_state']['joints']):raise ValueError('actual_stopped_open_arm_required')
   runtime.save(s,'before-empty-retraction');runtime.raise_open();runtime.tuck_open()
   result.update(status='empty_open_tucked_verified',state=runtime.state(),robot_commands_sent=True)
   if not pose_matches(result['state']['joints'],[0,0,0,-1.5707963267948966,0,.65]):raise ValueError('empty_tucked_not_observed')
  except Exception as e:result.update(status='stopped_requires_actual_state_review',reason=str(e))
  finally:
   if runtime:runtime.shutdown()
   rclpy.shutdown();(root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
if result['status']!='empty_open_tucked_verified':raise SystemExit(1)
