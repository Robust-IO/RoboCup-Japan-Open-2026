import sys,json,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from stopped_capture_runtime import with_stopped_capture
from horizontal_can_trial import HorizontalCanRuntime
from apple_transfer_automation import RobotLock
lock=RobotLock();lock.__enter__()
import rclpy
root=b/('controller-hand-check-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir();rclpy.init();r=with_stopped_capture(HorizontalCanRuntime,profile='paired')(root,'controller-hand-check',lambda e:None)
try:
 r.sample();goal=r.state()['joints'].copy()
 if goal[5]<.64:raise ValueError('open_hand_required')
 goal[0]=.51571187;r.move_joints(goal,'raise_open_above_controller')
 item=r.sample();cap=r.save(item,'paired');print(json.dumps(dict(capture=str(cap),state=r.state())));(b/'controller-hand-check-latest.json').write_text(json.dumps(dict(capture=str(cap),output=str(root))))
finally:r.shutdown();rclpy.shutdown();lock.__exit__(None,None,None)
