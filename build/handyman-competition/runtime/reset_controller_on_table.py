import sys,json,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from stopped_capture_runtime import with_stopped_capture
from horizontal_can_trial import HorizontalCanRuntime
import rclpy
root=b/('controller-replace-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir();report=dict(status='starting');r=None
with RobotLock():
 log=(root/'journal.jsonl').open('w')
 def record(e):log.write(json.dumps(e)+'\n');log.flush()
 try:
  rclpy.init();r=with_stopped_capture(HorizontalCanRuntime,profile='paired')(root,'controller-replace',record);r.sample();state=r.state();prior=json.loads((b/'controller-overhead-handle-20261107-173233/result.json').read_text())['last_state']
  if max(abs(a-c) for a,c in zip(state['joints'],prior['joints']))>.025:raise ValueError('held_posture_changed_reobserve_before_release')
  goal=state['joints'].copy();goal[0]-=.07;r.move_joints(goal,'lower_held_controller_back_to_source');r.save(r.sample(),'before-release');goal=r.state()['joints'].copy();goal[5]=.65;r.move_joints(goal,'release_on_source_table',hand=True);r.pump(.5);goal=r.state()['joints'].copy();goal[0]+=.15;r.move_joints(goal,'raise_open_clear_of_controller');r.pump(.3);cap=r.save(r.sample(),'released-and-raised');report.update(status='released_and_open_hand_raised',capture=str(cap),state=r.state())
 except BaseException as e:report.update(status='stopped',reason=str(e))
 finally:
  if r:r.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);write_json(b/'controller-replace-latest.json',dict(output=str(root),**report));log.close();print(json.dumps(report))
