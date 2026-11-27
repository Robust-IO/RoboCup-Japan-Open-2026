import sys,json,time,uuid,argparse
from pathlib import Path
import numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from stopped_capture_runtime import with_stopped_capture
from pepper_runtime import PepperRuntime
from outcome_transfer_runtime import OutcomeBackend
from round_body_geometry import measure
from hand_camera_ray import rotation
from apple_transfer_auto_plan import map_point,plan_placement
from can_destination_surface import measure as measure_table
from pepper_place import extension_clearance
from pregrasp_geometry import expected_horizontal_center
from tabletop_support import support_from_plan
from table_transfer_core import OFFSET,extension_retreat
from task_outcome_policy import RELEASE_BOTTOM_GAP_M
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=a.output
held=json.loads((b/'generic-round-grasp-20261107-170503/held-context.json').read_text());ref=held['reference'];holding=held['holding_opening_rad'];old=held['snapshot'];om=held['measurement']
# Original lift was performed in the standard flat orientation.
offset=np.asarray(om['center_m'])-np.asarray(old['palm_base_at_depth'])
hgeom=dict(center_minus_palm_m=offset.tolist(),radius_m=max(ref['max_radius_m'],om['max_radius_m']),holding_opening_rad=holding,bottom_below_palm_m=float(old['palm_base_at_depth'][2]-om['bottom_z_m']))
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());cfg['destination_room']='kitchen';cfg['destination_id']='dining_table#kitchen';cfg['joint_feedback_max_age_s']=.6
T=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==cfg['destination_id'])
report=dict(status='starting',target='tumbler',destination=T['id'],placement_success_verified=False,release_commands_sent=0);runtime=backend=None
with RobotLock():
 root.mkdir();log=(root/'journal.jsonl').open('w')
 def record(e):
  log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
 def stage(v):report['stage']=v;record(dict(event='stage',stage=v))
 try:
  import rclpy
  rclpy.init();runtime=with_stopped_capture(PepperRuntime)(root,'round-place-'+uuid.uuid4().hex,record);backend=OutcomeBackend(cfg);backend.bind(root,runtime.epoch,record);backend.observation_runtime=runtime
  runtime.sample();state=runtime.state()
  if abs(state['joints'][5]-holding)>.03:raise ValueError('current_grip_changed')
  stage('carry_to_kitchen_dining_table');backend.stage='navigate_destination';backend.navigate('kitchen','kitchen',[9.562303161621093,2.9079999923706055,3.141592653589793],holding,'dining-navigation')
  stage('measure_table');runtime.face_source();sample=runtime.sample();s=sample['snapshot'];plane=measure_table(sample['depth'],s,T);runtime.save(sample,'destination-table')
  table=runtime.table_geometry(sample,T,plane);plan=plan_placement(sample['depth'],s,table,plane,hgeom,cfg,runtime.epoch)
  for i in range(5):
   clearance=extension_clearance(map_point(s,expected_horizontal_center(plan['palm_height_m'],offset)),s,plane,plan)
   if clearance['ready']:break
   st=runtime.state();backend.stage='withdraw';backend.adjust('extension-standoff-'+str(i),st['poses']['map'],[-extension_retreat(clearance),0.,0.],st['joints'][:5],holding)
   sample=runtime.sample();s=sample['snapshot'];plane=measure_table(sample['depth'],s,T);plan=plan_placement(sample['depth'],s,table,plane,hgeom,cfg,runtime.epoch)
  else:raise ValueError('extension_clearance_unresolved')
  write_json(root/'placement-plan.json',plan);stage('extend_at_measured_table_height');runtime.prepare_place(plan['palm_height_m'],holding)
  serial=0
  def observe(label,expected_map=None):
   global serial
   item=runtime.sample();ss=item['snapshot'];palm=ss['palm_base_at_depth'];expected=np.asarray(palm)+offset
   if expected_map is not None:
    tf=ss['tracking_transforms']['map_from_base_at_depth'];expected=rotation(tf['quaternion']).T@(np.asarray(expected_map)-tf['translation'])
   try:m=measure(item['rgb'],item['depth'],ss,reference=ref,expected=expected)
   except Exception:
    runtime.save(item,'rejected-'+str(serial));raise
   plane=measure_table(item['depth'],ss,T);runtime.fresh_measurement(ss);serial+=1;runtime.save(item,label+'-'+str(serial),m)
   c=map_point(ss,np.asarray(m['center_m']));bottom=map_point(ss,np.r_[m['center_m'][:2],m['bottom_z_m']]);gap=float(bottom[2]-np.r_[bottom[:2],1.]@plane['plane_map']);edge=support_from_plan(plan).query(c[:2],hgeom['radius_m'])
   return item,m,dict(center_map=c.tolist(),bottom_gap_m=gap,edge=edge,ready=bool(edge['safe'] and RELEASE_BOTTOM_GAP_M[0]<=gap<=RELEASE_BOTTOM_GAP_M[1]))
  stage('approach_clear_patch')
  for i in range(11):
   item,m,check=observe('approach');st=runtime.state();record(dict(event='actual_release_geometry',**check))
   if check['ready']:break
   if not RELEASE_BOTTOM_GAP_M[0]<=check['bottom_gap_m']<=RELEASE_BOTTOM_GAP_M[1]:
    goal=st['joints'].copy();goal[0]+=cfg['policy']['release_gap_m']-check['bottom_gap_m'];runtime.move_joints(goal,'correct-release-height');continue
   delta=rotation(item['snapshot']['tracking_transforms']['map_from_base_at_depth']['quaternion']).T@np.r_[np.asarray(plan['release_xy_map_m'])-check['center_map'][:2],0.]
   backend.stage='approach_placement';backend.adjust('place-step-'+str(i),st['poses']['map'],[float(delta[0]),float(delta[1]),0.],st['joints'][:5],holding,single_placement_approach=True)
  else:raise ValueError('placement_not_ready')
  release={}
  def verify_release():
   item,m,check=observe('pre-release')
   if not check['ready']:raise ValueError('current_support_not_ready')
   release.update(check);runtime.fresh_measurement(item['snapshot'])
  stage('release');runtime.release(verify_release);report['release_commands_sent']=1;write_json(root/'release.json',release)
  stage('withdraw_empty');runtime.raise_open();st=runtime.state();backend.stage='withdraw';backend.adjust('empty-retreat',st['poses']['map'],[-cfg['policy']['retreat_m'],0.,0.],st['joints'][:5],st['joints'][5]);runtime.tuck_open()
  stage('verify_placed');rows=[]
  expected=np.array(release['center_map']);expected[2]-=release['bottom_gap_m']
  for i in range(3):
   item,m,check=observe('placed',expected)
   if abs(check['bottom_gap_m'])>.035 or not check['edge']['safe']:raise ValueError('actual_cup_not_supported')
   rows.append(check);runtime.pump(.25)
  if np.max(np.linalg.norm(np.asarray([v['center_map'] for v in rows])-np.mean([v['center_map'] for v in rows],axis=0),axis=1))>.025:raise ValueError('cup_not_stable')
  report.update(placement_success_verified=True,placed_frames=rows);stage('return_for_game_controller');backend.stage='navigate_source';backend.navigate('kitchen','kitchen',[9.205,1.482962,-.012512],runtime.state()['joints'][5],'return-white-table')
  runtime.face_source();sample=runtime.sample();runtime.save(sample,'controller-source');report.update(status='cup_placed_returned_for_controller',controller_grasped=False)
 except BaseException as e:report.update(status='stopped_requires_current_state_review',reason=str(e));record(dict(event='error',reason=str(e)))
 finally:
  if backend:backend.close()
  if runtime:
   try:report['last_state']=runtime.state()
   except Exception:pass
   runtime.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
