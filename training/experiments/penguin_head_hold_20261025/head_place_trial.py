"""Measured-table placement of this RGB-verified rigid grasp.

Held object pose is explicitly a kinematic estimate anchored by preclose RGBD;
postrelease success requires NEW independent body+head depth and support fits.
"""
import argparse,json,sys,time,uuid
from pathlib import Path
import cv2,numpy as np,rclpy
sys.path.insert(0,'/mnt/d/handyman-tools/apple-transfer-auto-v1')
from penguin_runtime import PenguinRuntime
from penguin_hold import expected_centers
from penguin_geometry import surface_extent,measure_near
from penguin_place import make_plan,extension_clearance,region,approach_step,expected_placed,verify_placed
from can_destination_surface import measure as measure_table
from apple_transfer_auto_plan import map_point
from apple_transfer_auto_backend import LiveBackend
from apple_transfer_automation import RobotLock
from task_outcome_policy import pose_matches,RELEASE_BOTTOM_GAP_M,robot_ready
from inverse_table_place_plan import OFFSET
from pregrasp_geometry import expected_horizontal_center
ap=argparse.ArgumentParser();ap.add_argument('--run',action='store_true');args=ap.parse_args()
base=Path('/mnt/d/handyman-training-runs');prior=base/'penguin-head-carry-test-20261025-01'
p=json.loads((prior/'result.json').read_text());meta=json.loads((prior/'run-meta.json').read_text());held=json.loads((prior/'held-context.json').read_text());cfg=json.loads((prior/'config.snapshot.json').read_text())
assert p['status']=='penguin_destination_hold_verified' and p['carry_destination_verified']
assert meta['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip()
model=held['model'];holding=held['holding_opening_rad'];ref=held['hand_face_reference'];x1,y1,x2,y2=ref['box']
template=cv2.imread(str(Path(ref['capture'])/'hand.png'))[y1:y2,x1:x2]
destination=next(x for x in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if x['id']==cfg['destination_id'])
root=base/('penguin-head-place-test-20261025-01' if args.run else 'penguin-head-place-plan-20261025-02')
run_id='penguin-head-place-'+uuid.uuid4().hex[:10]
result=dict(class_name='toy_penguin',status='starting',close_commands_sent=0,release_commands_sent=0,
    prior_close_commands_sent=1,predecessor_run=str(prior),grasp_success_verified=True,
    placement_success_verified=False,robot_commands_sent=False,single_uninterrupted_automatic_run=False,
    held_pose_source='preclose_rgbd_anchored_rigid_shape_with_current_hand_face_rgb',task_finished_sent=False)
events=[];runtime=None
def save(): (root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False))
def record(e):
 events.append(e)
 if e.get('event')=='pepper_joint_command_sent':
  result['robot_commands_sent']=True
  if e.get('label')=='release_once':result['release_commands_sent']+=1
  save()
def face(frame):
 sy,sx=max(0,y1-32),max(0,x1-32);search=frame[sy:min(frame.shape[0],y2+32),sx:min(frame.shape[1],x2+32)]
 _,score,_,where=cv2.minMaxLoc(cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED))
 x,y=sx+where[0],sy+where[1];patch=frame[y:y+template.shape[0],x:x+template.shape[1]]
 count=int(np.count_nonzero(cv2.inRange(cv2.cvtColor(patch,cv2.COLOR_BGR2HSV),np.array([15,90,50]),np.array([45,255,255]))))
 shift=[x-x1,y-y1]
 if score<.8 or count<.5*ref['reference_yellow_pixels'] or np.linalg.norm(shift)>24:
  raise ValueError('current_hand_face_not_at_verified_grip')
 return dict(correlation=score,yellow_pixels=count,pixel_shift=shift,new_3d_observation=False)
serial=0
def observe(label):
 global serial
 sample=runtime.sample();s=sample['snapshot'];state=runtime.state()
 if abs(state['joints'][5]-holding)>.03:raise ValueError('current_grip_changed')
 appearance=face(sample['hand']);palm=runtime.palm_pose(s);b,h=expected_centers(palm,held)
 shape=surface_extent(b,h,model['body_radius_m'],model['cap_radius_m'],model['body_above_support_m'])
 shape['horizontal_support_radius_m']=max(shape['horizontal_support_radius_m'],model['observed_horizontal_extent_m'])
 estimate=dict(class_name='toy_penguin',center_m=b.tolist(),cap_center_m=h.tolist(),
    body_radius_m=model['body_radius_m'],cap_radius_m=model['cap_radius_m'],**shape,
    height_m=model['height_m'],body_above_support_m=model['body_above_support_m'],
    stamp_ns=s['streams']['depth']['stamp_ns'],kinematic_state_evaluation_stamp_ns=s['streams']['depth']['stamp_ns'],
    original_object_depth_stamp_ns=held['last_horizontal_measurement']['stamp_ns'],
    current_body_depth_measured=False,current_head_depth_measured=False,predicted_center_returned=True,
    position_source='held_palm_kinematic_estimate_with_current_hand_face_rgb',hidden_surfaces_measured=False)
 plane=measure_table(sample['depth'],s,destination);runtime.fresh_measurement(s)
 serial+=1;capture=runtime.save(sample,str(serial).zfill(3)+'-'+label,estimate)
 return dict(measurement=estimate,snapshot=s,plane=plane,state=state,capture=str(capture),hand_face=appearance),sample
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();runtime=PenguinRuntime(root,run_id,record);backend=LiveBackend(cfg);backend.bind(root,run_id,record)
 try:
  runtime.sample();state=runtime.state();runtime.ownership()
  assert pose_matches(state['joints'],p['final_state']['joints'])
  assert backend.room_at(state['poses']['map'])==cfg['destination_room']
  row,sample=observe('table-before-extension');s=row['snapshot']
  plan=make_plan(sample['depth'],s,destination,row['plane'],held,cfg,run_id)
  predicted=map_point(s,expected_horizontal_center(plan['palm_height_m'],held['body_minus_palm_horizontal_m']))
  clearance=extension_clearance(predicted,s,row['plane'],plan)
  assert clearance['ready'],clearance
  result.update(plan=plan,extension_clearance=clearance,stage='plan_ready',current_hand_face=row['hand_face']);save()
  if args.run:
   result['stage']='prepare_horizontal_for_placement';save();runtime.prepare_place(plan['palm_height_m'],holding)
   result['stage']='approach_modelled_release_region';save();steps=[]
   for index in range(6):
    row,_=observe('approach');check=region(row['measurement'],row['snapshot'],plan,row['plane'])
    if check['ready']:break
    if index==5:raise ValueError('five_placement_corrections_exhausted')
    if not RELEASE_BOTTOM_GAP_M[0]<=check['estimated_bottom_gap_m']<=RELEASE_BOTTOM_GAP_M[1]:
     goal=row['state']['joints'].copy();goal[0]+=cfg['policy']['release_gap_m']-check['estimated_bottom_gap_m']
     runtime.move_joints(goal,'correct_modelled_release_height');plan['palm_height_m']=goal[0]+OFFSET
     steps.append(dict(kind='height',lift=goal[0]));continue
    step=approach_step(row['measurement'],row['snapshot'],plan,row['plane']);backend.stage='approach_placement'
    backend.adjust('penguin-placement-'+str(index),row['state']['poses']['map'],step['delta_m'],row['state']['joints'][:5],holding,single_placement_approach=True)
    steps.append(dict(kind='base',delta=step['delta_m']))
   result['steps']=steps;result['stage']='release_once';save()
   def release_check():
    row,_=observe('pre-release');check=region(row['measurement'],row['snapshot'],plan,row['plane'])
    if not check['ready']:raise ValueError('estimated_held_footprint_not_over_current_safe_table_support')
    b=map_point(row['snapshot'],np.asarray(row['measurement']['center_m']));h=map_point(row['snapshot'],np.asarray(row['measurement']['cap_center_m']))
    plan.update(actual_release_xy_map_m=b[:2].tolist(),actual_cap_minus_body_map_m=(h-b).tolist(),
                release_xy_source='kinematic_held_pose_estimate_not_new_object_depth')
    result['release_support']=dict(check=check,hand_face=row['hand_face'],capture=row['capture'],object_pose_independently_depth_measured=False)
    runtime.fresh_measurement(row['snapshot']);save()
   runtime.release(release_check);result['hand_released']=True;result['plan']=plan;save()
   result['stage']='withdraw_empty';save();runtime.raise_open();state=runtime.state();backend.stage='withdraw'
   backend.adjust('penguin-placement-retreat',state['poses']['map'],[-cfg['policy']['retreat_m'],0.,0.],state['joints'][:5],state['joints'][5]);runtime.tuck_open()
   result['stage']='verify_new_actual_placed_rgbd';save();placed=[]
   for index in range(12):
    sample=runtime.sample();s=sample['snapshot']
    try:
     b,h=expected_placed(s,plan);m=measure_near(sample['rgb'],sample['depth'],s,b,h,model)
     if not m.get('current_body_depth_measured') or not m.get('current_head_depth_measured'):
      raise ValueError('independent_actual_body_and_head_depth_required_after_release')
     plane=measure_table(sample['depth'],s,destination);runtime.fresh_measurement(s)
     if placed and m['stamp_ns']-placed[-1]['measurement']['stamp_ns']<200_000_000:runtime.pump(.2);continue
     capture=runtime.save(sample,'placed-'+str(index),m);placed.append(dict(measurement=m,snapshot=s,plane=plane,capture=str(capture)))
     if len(placed)==3:break
    except ValueError as exc:
     runtime.save(sample,'placed-rejected-'+str(index));events.append(dict(event='placed_depth_rejected',reason=str(exc)))
   state=runtime.state();verification=verify_placed(placed,plan,state['joints'],cleanup_completed=True)
   result.update(status='penguin_staged_transfer_verified',placement_success_verified=True,physical_transfer_verified=True,
                 placement_verification=verification,placed_frames=placed,final_state=state)
  else:result.update(status='measured_table_kinematic_place_plan_ready',robot_commands_sent=False)
 except Exception as exc:result.update(status='stopped_requires_actual_state_review',reason=str(exc),final_state=runtime.state())
 finally:
  result['events']=events;result['completed_ns']=time.time_ns();save();runtime.shutdown();rclpy.shutdown()
  (root/'held-context.json').write_text(json.dumps(held,indent=2));(root/'config.snapshot.json').write_bytes((prior/'config.snapshot.json').read_bytes())
  (root/'run-meta.json').write_text(json.dumps(dict(run_id=run_id,run_root=str(root),boot_id=meta['boot_id'],object='toy_penguin',task_sha256=meta['task_sha256']),indent=2))
  print(json.dumps({k:v for k,v in result.items() if k not in ('events','placed_frames')}),flush=True)
