"""Recover only the verified still-open empty arm after the stopped source approach."""
import json,sys,uuid
from pathlib import Path
import rclpy
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
from penguin_runtime import PenguinRuntime
from apple_transfer_automation import RobotLock
from task_outcome_policy import hand_is_open
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');previous=json.loads((base/'penguin-native-transfer-20261025-02/result.json').read_text())
assert previous['close_commands_sent']==previous['release_commands_sent']==0
root=base/'penguin-empty-arm-recovery-20261025-02'
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();events=[]
 runtime=PenguinRuntime(root,'penguin-empty-recovery-'+uuid.uuid4().hex[:8],events.append)
 result=dict(close_commands_sent=0,release_commands_sent=0,base_commands_sent=False)
 try:
  runtime.sample();state=runtime.state();assert hand_is_open(state['joints'][5])
  assert abs(state['joints'][0]-.32204)<.01 and abs(state['joints'][1]+1.7)<.02
  result['initial_state']=state
  runtime.raise_open();runtime.tuck_open();runtime.face_source([0.,-.5])
  result.update(status='empty_arm_recovered',final_state=runtime.state())
 except Exception as exc:result.update(status='stopped',reason=str(exc))
 finally:
  runtime.shutdown();rclpy.shutdown();result['events']=events
  (root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
  print(json.dumps({k:v for k,v in result.items() if k!='events'}),flush=True)
