import sys,json,time,uuid
from pathlib import Path
import numpy as np,cv2
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from stopped_capture_runtime import with_stopped_capture
from horizontal_can_trial import HorizontalCanRuntime
from outcome_transfer_runtime import OutcomeBackend
p=b/'autograsp-car-locked-overhead-20261107-02';prior=json.loads((p/'result.json').read_text());holding=prior['last_state']['joints'][5];root=b/('autograsp-car-dining-carry-'+time.strftime('%Y%m%d-%H%M%S'));cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());report=dict(status='starting',target='toy_car',destination='dining_table#kitchen',release_commands_sent=0);r=backend=None
with RobotLock():
 root.mkdir();log=(root/'journal.jsonl').open('w');write_json(b/'autograsp-car-dining-carry-latest.json',dict(output=str(root)))
 def record(e):log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
 def stage(v):report['stage']=v;record(dict(event='stage',stage=v))
 try:
  import rclpy
  rclpy.init();r=with_stopped_capture(HorizontalCanRuntime,profile='paired')(root,'controller-carry-'+uuid.uuid4().hex,record);r.sample();st=r.state()
  if abs(st['joints'][5]-holding)>.03:raise ValueError('grip_changed')
  backend=OutcomeBackend(cfg);backend.bind(root,r.epoch,record);backend.observation_runtime=r
  r.save(r.sample(),'before-carry');stage('withdraw_before_folding');backend.stage='withdraw';backend.adjust('held-withdraw',st['poses']['map'],[-.40,0.,0.],st['joints'][:5],holding)
  stage('fold_holding');r.prepare_navigation_from_current();r.save(r.sample(),'held-folded')
  stage('navigate_dining');backend.stage='navigate_destination';backend.navigate('kitchen','kitchen',[9.562303161621093,2.9079999923706055,3.141592653589793],holding,'dining-navigation');r.face_source();r.save(r.sample(),'dining-table');report.update(status='arrived_grip_preserved_car',holding_opening_rad=r.state()['joints'][5],last_state=r.state())
 except BaseException as e:report.update(status='stopped',reason=str(e))
 finally:
  if backend:backend.close()
  if r:r.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
