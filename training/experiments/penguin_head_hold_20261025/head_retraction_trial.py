"""Keep this already lifted grasp; test folding without opening or reclosing."""
import json,sys,time,uuid
from pathlib import Path
import cv2,numpy as np,rclpy
sys.path.insert(0,'/mnt/d/handyman-tools/apple-transfer-auto-v1')
from penguin_runtime import PenguinRuntime
from penguin_hold import palm_points
from hand_camera_ray import rotation
from scipy.spatial.transform import Rotation
from apple_transfer_automation import RobotLock
from task_outcome_policy import pose_matches
base=Path('/mnt/d/handyman-training-runs');prior=base/'penguin-native-transfer-20261025-10';lift=base/'penguin-current-head-lift-test-20261025-01'
p=json.loads((prior/'result.json').read_text());l=json.loads((lift/'result.json').read_text());meta=json.loads((prior/'run-meta.json').read_text())
assert meta['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip()
assert p['close_commands_sent']==1 and p['release_commands_sent']==0 and l['robot_commands_sent']
audit=json.loads((base/'penguin-hand-face-lift-follow-audit-20261025.json').read_text());box=audit['template_box'];x1,y1,x2,y2=box
template=cv2.imread(str(Path(audit['reference'])/'hand.png'))[y1:y2,x1:x2]
def face(frame):
 sy,sx=max(0,y1-32),max(0,x1-32);search=frame[sy:min(frame.shape[0],y2+32),sx:min(frame.shape[1],x2+32)]
 match=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED);_,score,_,where=cv2.minMaxLoc(match)
 x,y=sx+where[0],sy+where[1];patch=frame[y:y+template.shape[0],x:x+template.shape[1]]
 yellow=cv2.inRange(cv2.cvtColor(patch,cv2.COLOR_BGR2HSV),np.array([15,90,50]),np.array([45,255,255]))
 count=int(np.count_nonzero(yellow));shift=[x-x1,y-y1]
 return dict(verified=bool(score>=.8 and count>=1000 and np.linalg.norm(shift)<=24),
             correlation=score,yellow_pixels=count,pixel_shift=shift,measured_3d_position_available=False)
root=base/'penguin-head-retraction-test-20261025-01';run_id='penguin-head-retract-'+uuid.uuid4().hex[:10]
holding=p['close']['closure_evidence']['observed_opening_rad'];result=dict(class_name='toy_penguin',
 close_commands_sent=0,release_commands_sent=0,prior_close_commands_sent=1,base_commands_sent=False,
 predecessor_run=str(prior),lift_run=str(lift),rows=[],robot_commands_sent=False,grasp_success_verified=False)
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();events=[];runtime=PenguinRuntime(root,run_id,events.append)
 try:
  first=runtime.sample();state=runtime.state();runtime.ownership()
  assert pose_matches(state['joints'],l['final_state']['joints'])
  assert np.linalg.norm(np.asarray(state['poses']['odom'])[:2]-l['final_state']['poses']['odom'][:2])<.03
  evidence=face(first['hand']);assert evidence['verified'],evidence
  result['before_retraction']=evidence;runtime.save(first,'before-retraction')
  runtime.retract_holding(holding);result['robot_commands_sent']=True
  last=0
  while len(result['rows'])<3:
   sample=runtime.sample();s=sample['snapshot'];stamp=s['streams']['hand']['stamp_ns']
   if stamp-last<200_000_000:runtime.pump(.2);continue
   evidence=face(sample['hand']);evidence.update(hand_stamp_ns=stamp,
       capture=str(runtime.save(sample,'after-retraction-'+str(len(result['rows'])))))
   result['rows'].append(evidence);last=stamp
  okay=all(row['verified'] for row in result['rows'])
  result.update(status='head_grasp_retraction_verified' if okay else 'retracted_requires_visual_review',
      stage='head_grasp_retraction_verified',grasp_success_verified=okay,
      final_state=runtime.state(),object_remains_held=True,
      proof_type='measured_10cm_palm_lift_and_persistent_target_face_relative_to_hand_plus_preserved_grip')
  if okay:
   before_path=Path(p['close']['before_capture']);bs=json.loads((before_path/'snapshot.json').read_text());m=json.loads((before_path/'object-measurement.json').read_text())
   bh,ph=bs['base_from_hand'],bs['palm_from_hand'];r=rotation(bh['quaternion'])@rotation(ph['quaternion']).T
   palm=dict(translation=(np.asarray(bh['translation'])-r@ph['translation']).tolist(),quaternion=Rotation.from_matrix(r).as_quat().tolist())
   bp,hp=palm_points(m['center_m'],m['cap_center_m'],palm)
   held=dict(class_name='toy_penguin',run_id=run_id,run_root=str(root),boot_id=meta['boot_id'],
       grasp_lift_verified=True,holding_opening_rad=holding,model=p['source_observations'][-1]['measurement'],
       body_palm_m=bp,cap_palm_m=hp,body_minus_palm_horizontal_m=(np.asarray(m['center_m'])-bs['palm_base_at_depth']).tolist(),
       last_horizontal_measurement=m,body_palm_source='preclose_rgbd_registered_shape_with_current_hand_rgb_rigidity',
       head_palm_source='preclose_rgbd_with_current_hand_rgb_rigidity',
       current_held_xyz_independently_depth_measured=False,original_grasp_run=str(prior),
       hand_face_reference=dict(capture=audit['reference'],box=box,reference_yellow_pixels=2194),
       hand_rgb_lift_audit=str(base/'penguin-hand-face-lift-follow-audit-20261025.json'))
   (root/'held-context.json').write_text(json.dumps(held,indent=2,allow_nan=False))
 except Exception as exc:result.update(status='stopped_requires_actual_state_review',reason=str(exc))
 finally:
  result['events']=events;result['completed_ns']=time.time_ns();runtime.shutdown();rclpy.shutdown()
  (root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
  (root/'config.snapshot.json').write_bytes((prior/'config.snapshot.json').read_bytes())
  (root/'run-meta.json').write_text(json.dumps(dict(run_id=run_id,run_root=str(root),boot_id=meta['boot_id'],object='toy_penguin',task_sha256=p['task_sha256']),indent=2))
  print(json.dumps({k:v for k,v in result.items() if k!='events'}),flush=True)
