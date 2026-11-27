import json,uuid
from pathlib import Path
import cv2,rclpy
from horizontal_can_hand_trial import HandCanRuntime
from apple_transfer_automation import RobotLock,write_json
from flat_box_transport import shape,silhouette
from flat_box_geometry import measure_near
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'cigarette-place-review-20261101-01'
held=json.loads((base/'cigarette-carry-20261101-02/held-context.json').read_text())
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();runtime=HandCanRuntime(root,'box-review-'+uuid.uuid4().hex,lambda e:None)
 try:
  sample=runtime.sample(processing_reserve_ns=0);capture=runtime.save(sample,'current')
  estimate=shape(sample['snapshot'],held);out=dict(robot_commands_sent=False,state=runtime.state(),held_estimate=estimate,capture=str(capture))
  try:out['actual_head_box']=measure_near(sample['rgb'],sample['depth'],sample['snapshot'],estimate['center_m'],held['model'],held['appearance'])
  except ValueError as e:out['head_reason']=str(e)
  try:out['hand_proof']=silhouette(cv2.imread(str(base/'cigarette-grasp-20261101-01/captures/lift-proof-2/hand.png')),sample['hand'],held['appearance'])
  except ValueError as e:out['hand_reason']=str(e)
  write_json(root/'result.json',out);print(json.dumps(out))
 finally:runtime.shutdown();rclpy.shutdown()
