"""Operator-requested release, then empty retreat and tuck; no grasp replay."""
import json,math,time,uuid,signal
from pathlib import Path
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import LiveBackend
from horizontal_can_hand_trial import HandCanRuntime
from task_outcome_policy import pose_matches
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'cigarette-operator-reset-20261101-01'
cfg=json.loads((base/'cigarette-carry-20261101-02/config.snapshot.json').read_text());cfg['joint_feedback_max_age_s']=.6
prior=json.loads((base/'cigarette-place-review-20261101-01/result.json').read_text())
report=dict(status='starting',operator_requested_release=True,close_commands_sent=0,release_commands_sent=0)
runtime=backend=None
def stop(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,stop)
with RobotLock():
 root.mkdir(exist_ok=False);started=time.time_ns()
 with (root/'journal.jsonl').open('x') as log:
  def record(e):
   if e.get('event')=='horizontal_joint_sent' and e.get('label')=='operator_requested_release':report['release_commands_sent']+=1
   log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
  try:
   import rclpy
   rclpy.init();epoch='box-reset-'+uuid.uuid4().hex;runtime=HandCanRuntime(root,epoch,record);backend=LiveBackend(cfg);backend.bind(root,epoch,record)
   runtime.sample(processing_reserve_ns=0);state=runtime.state();runtime.ownership()
   if not pose_matches(state['joints'],prior['state']['joints']):raise ValueError('current_held_arm_state_changed')
   goal=state['joints'];goal[5]=.65;runtime.move_joints(goal,'operator_requested_release',hand=True)
   report['stage']='withdraw_empty';write_json(root/'result.json',report)
   runtime.raise_open();state=runtime.state();backend.stage='withdraw'
   backend.adjust('operator-reset-empty-retreat',state['poses']['map'],[-.60,0.,0.],state['joints'][:5],state['joints'][5])
   runtime.tuck_open();sample=runtime.sample(processing_reserve_ns=0);runtime.save(sample,'ready-for-operator-placement')
   report.update(status='open_empty_tucked_waiting_for_operator_object_placement',final_state=runtime.state(),placement_success_claimed=False)
  except (Exception,KeyboardInterrupt) as e:
   report.update(status='stopped_requires_actual_state_review',reason=str(e) or 'interrupted')
   if runtime:
    try:report['last_robot_state']=runtime.state()
    except Exception:pass
  finally:
   if backend:backend.close()
   if runtime:runtime.shutdown();rclpy.shutdown()
   report['elapsed_s']=(time.time_ns()-started)/1e9;write_json(root/'result.json',report);print(json.dumps(report))
