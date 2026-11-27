import json,time,uuid
from pathlib import Path
import rclpy
from apple_transfer_automation import RobotLock,write_json
from horizontal_can_hand_trial import HandCanRuntime
from task_outcome_policy import pose_matches,hand_is_open
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'cigarette-operator-reset-completion-20261101-01'
prior=json.loads((base/'cigarette-operator-reset-20261101-01/result.json').read_text());report=dict(status='starting',close_commands_sent=0,release_commands_sent=0,previous_release_commands_sent=prior['release_commands_sent'])
if prior['release_commands_sent']!=1:raise ValueError('previous_release_required')
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();runtime=HandCanRuntime(root,'empty-reset-'+uuid.uuid4().hex,lambda e:None)
 try:
  runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
  if not hand_is_open(state['joints'][5]) or not pose_matches(state['joints'],prior['last_robot_state']['joints']):raise ValueError('actual_released_open_state_required')
  runtime.base_move([-.60,0.,0.],state['joints'][:5],'single_empty_retreat_for_operator');runtime.tuck_open()
  sample=runtime.sample(processing_reserve_ns=0);runtime.save(sample,'ready-for-operator-placement')
  report.update(status='open_empty_tucked_waiting_for_operator_object_placement',final_state=runtime.state())
 except Exception as e:report.update(status='stopped_requires_actual_state_review',reason=str(e),last_robot_state=runtime.state())
 finally:runtime.shutdown();rclpy.shutdown();write_json(root/'result.json',report);print(json.dumps(report))
