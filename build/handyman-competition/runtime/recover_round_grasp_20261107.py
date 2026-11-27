import json,time,sys
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from apple_transfer_automation import RobotLock
from stopped_capture_runtime import with_stopped_capture
from pepper_runtime import PepperRuntime
from outcome_transfer_runtime import OutcomeBackend
import rclpy
root=b/('round-open-retreat-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir()
with RobotLock():
 rclpy.init();r=with_stopped_capture(PepperRuntime)(root,'round-reset',lambda e:print(json.dumps(e),flush=True))
 try:
  r.sample();s=r.state();assert abs(s['joints'][5]-.65)<.02
  cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());backend=OutcomeBackend(cfg);backend.bind(root,'round-reset',lambda e:None);backend.observation_runtime=r;backend.stage='withdraw'
  backend.adjust('withdraw-open',s['poses']['map'],[-.35,0.,0.],s['joints'][:5],s['joints'][5]);r.tuck_open()
  print(json.dumps(dict(status='empty_retracted',state=r.state())))
 finally:r.shutdown();rclpy.shutdown()
