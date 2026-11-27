"""Carry the already verified head grasp on the unchanged furniture route."""
import json,sys,time,uuid
from pathlib import Path
import cv2,numpy as np,rclpy
sys.path.insert(0,'/mnt/d/handyman-tools/apple-transfer-auto-v1')
from penguin_runtime import PenguinRuntime
from apple_transfer_auto_backend import LiveBackend
from apple_transfer_automation import RobotLock
from task_outcome_policy import pose_matches
base=Path('/mnt/d/handyman-training-runs');prior=base/'penguin-head-retraction-test-20261025-01'
p=json.loads((prior/'result.json').read_text());meta=json.loads((prior/'run-meta.json').read_text())
held=json.loads((prior/'held-context.json').read_text());cfg=json.loads((prior/'config.snapshot.json').read_text())
assert p['status']=='head_grasp_retraction_verified' and p['grasp_success_verified']
assert meta['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip()
reference=held['hand_face_reference'];x1,y1,x2,y2=reference['box']
template=cv2.imread(str(Path(reference['capture'])/'hand.png'))[y1:y2,x1:x2]
def face(frame):
 sy,sx=max(0,y1-32),max(0,x1-32);search=frame[sy:min(frame.shape[0],y2+32),sx:min(frame.shape[1],x2+32)]
 _,score,_,where=cv2.minMaxLoc(cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED))
 x,y=sx+where[0],sy+where[1];patch=frame[y:y+template.shape[0],x:x+template.shape[1]]
 yellow=cv2.inRange(cv2.cvtColor(patch,cv2.COLOR_BGR2HSV),np.array([15,90,50]),np.array([45,255,255]))
 count=int(np.count_nonzero(yellow));shift=[x-x1,y-y1]
 return dict(verified=bool(score>=.8 and count>=.5*reference['reference_yellow_pixels'] and np.linalg.norm(shift)<=24),
     correlation=score,yellow_pixels=count,pixel_shift=shift,new_3d_depth_position_created=False)
root=base/'penguin-head-carry-test-20261025-01';run_id='penguin-head-carry-'+uuid.uuid4().hex[:10]
holding=held['holding_opening_rad'];result=dict(class_name='toy_penguin',stage='preflight_current_tucked_hold',
 close_commands_sent=0,release_commands_sent=0,prior_close_commands_sent=1,predecessor_run=str(prior),
 grasp_success_verified=True,carry_destination_verified=False,placement_success_verified=False,
 navigation_parameters_changed=False,rows=[],robot_commands_sent=False)
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();events=[]
 runtime=PenguinRuntime(root,run_id,events.append);backend=LiveBackend(cfg);backend.bind(root,run_id,events.append)
 try:
  sample=runtime.sample();state=runtime.state();runtime.ownership()
  assert pose_matches(state['joints'],p['final_state']['joints'])
  assert backend.room_at(state['poses']['map'])==cfg['source_room']
  check=face(sample['hand']);assert check['verified'],check
  result['before_carry']=check;runtime.save(sample,'before-carry')
  result['stage']='navigate_destination';backend.stage='navigate_destination'
  (root/'result.json').write_text(json.dumps(result,indent=2))
  backend.navigate(cfg['source_room'],cfg['destination_room'],cfg['destination_staging_goal'],holding,'destination-navigation')
  result['robot_commands_sent']=True;last=0
  while len(result['rows'])<3:
   sample=runtime.sample();s=sample['snapshot'];stamp=s['streams']['hand']['stamp_ns']
   if stamp-last<200_000_000:runtime.pump(.2);continue
   check=face(sample['hand']);check.update(hand_stamp_ns=stamp,capture=str(runtime.save(sample,'destination-held-'+str(len(result['rows'])))))
   result['rows'].append(check);last=stamp
  okay=all(row['verified'] for row in result['rows'])
  result.update(status='penguin_destination_hold_verified' if okay else 'destination_hold_requires_visual_review',
      stage='destination_hold_verified',carry_destination_verified=okay,final_state=runtime.state(),object_remains_held=True,
      original_grasp_run=held['original_grasp_run'])
  if okay:
   held.update(run_id=run_id,run_root=str(root),previous_held_context=str(prior/'held-context.json'))
   (root/'held-context.json').write_text(json.dumps(held,indent=2))
 except Exception as exc:result.update(status='stopped_requires_actual_state_review',reason=str(exc))
 finally:
  result['events']=events;result['completed_ns']=time.time_ns();runtime.shutdown();rclpy.shutdown()
  (root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
  (root/'config.snapshot.json').write_bytes((prior/'config.snapshot.json').read_bytes())
  (root/'run-meta.json').write_text(json.dumps(dict(run_id=run_id,run_root=str(root),boot_id=meta['boot_id'],object='toy_penguin',task_sha256=meta['task_sha256']),indent=2))
  print(json.dumps({k:v for k,v in result.items() if k!='events'}),flush=True)
