import sys,json,time,uuid,math
from pathlib import Path
import numpy as np,cv2
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from stopped_capture_runtime import with_stopped_capture
from horizontal_can_trial import HorizontalCanRuntime
from outcome_transfer_runtime import OutcomeBackend
from hand_camera_ray import rotation
from horizontal_can_grasp_plan import forward
from can_destination_surface import measure as measure_table
from apple_transfer_auto_plan import map_point,depth_in_map,footprint,clear_patch
from tabletop_support import build_support,TableSupport
from table_approach_reachability import base_path
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());T=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='dining_table#kitchen');prior=b/'autograsp-car-locked-overhead-20261107-02'
from local_handle_geometry import plane_base
ss=json.loads((prior/'captures/rejected-track-7/snapshot.json').read_text()); pp=ss['base_from_palm_at_depth']; cloud=np.load(prior/'captures/rejected-track-7/registration-inputs.npz')['template']; local=(cloud-pp['translation'])@rotation(pp['quaternion'])
pre=prior/'captures/preclose'; ps=json.loads((pre/'snapshot.json').read_text()); source=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='white_side_table#kitchen'); plane0=plane_base(ps,measure_table(np.load(pre/'depth_m.npy'),ps,source)['plane_map']); xy=np.asarray(json.loads((pre/'object-measurement.json').read_text())['center_m'])[:2]; bottom=np.r_[xy,xy@plane0[:2]+plane0[2]]; bp=ps['base_from_palm_at_depth']; bottomlocal=(bottom-bp['translation'])@rotation(bp['quaternion'])
holding=json.loads((prior/'result.json').read_text())['last_state']['joints'][5]
_,rr=forward([0,-math.pi/2,0,-math.pi/2,0,holding]); shape=local@rr.T; ctr=np.median(shape,axis=0); ct=rr@np.array([0,.00005,.062125866]); radius=float(np.max(np.linalg.norm(shape[:,:2]-ctr[:2],axis=1)))+.006
held=dict(holding_opening_rad=holding,radius_m=radius,center_minus_contact_xy=(ctr[:2]-ct[:2]).tolist(),bottom_below_palm_m=float(-(bottomlocal@rr.T)[2]),geometry_basis='previously_measured_shape_in_palm_frame_current_hold_visually_confirmed')
write_json(b/'autograsp-car-placement-context.json',held)
root=b/('autograsp-car-dining-place-'+time.strftime('%Y%m%d-%H%M%S'));report=dict(status='starting',target='toy_car',destination=T['id'],release_commands_sent=0,placement_success_verified=False);r=backend=None
def held_image(im):
 current=r.state()['joints'][5]
 if abs(current-holding)>.03:raise ValueError('held_grip_changed')
 return dict(grip_preserved=True,visual_hold_automatically_verified=False,reference='before-carry hand image reviewed; no current target-visibility requirement')

with RobotLock():
 root.mkdir();log=(root/'journal.jsonl').open('w');write_json(b/'autograsp-car-dining-place-latest.json',dict(output=str(root)))
 def record(e):log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
 def stage(v):report['stage']=v;record(dict(event='stage',stage=v))
 try:
  import rclpy
  rclpy.init();r=with_stopped_capture(HorizontalCanRuntime,profile='paired')(root,'controller-place-'+uuid.uuid4().hex,record);backend=OutcomeBackend(cfg);backend.bind(root,r.epoch,record);backend.observation_runtime=r
  r.face_source((0.,-.15));item=r.sample();s=item['snapshot'];st=r.state()
  if abs(st['joints'][5]-holding)>.03:raise ValueError('grip_changed')
  record(dict(event='actual_held_texture',proof=held_image(item['hand'])));r.save(item,'destination-table-held');stage('measure_clear_destination_patch');plane=measure_table(item['depth'],s,T);table=r.table_geometry(item,T,plane);center,half=footprint(table);points=depth_in_map(item['depth'],s);support=TableSupport(build_support(points,center,half,plane['plane_map'],T['id'],s['streams']['depth']['stamp_ns']),T['id'],edge_gap_m=.02);radius=held['radius_m'];positions=support.safe_centers(radius,0.);choices=[];mr=rotation(s['tracking_transforms']['map_from_base_at_depth']['quaternion']);mt=np.array(s['tracking_transforms']['map_from_base_at_depth']['translation'])
  elbow=-math.pi/2
  for flex in [-math.pi/2,-1.4,-1.25,-1.1]:
   z0,_=forward([0,flex,0,-math.pi-flex,0,holding])
   needed=float(np.r_[center,1.]@plane["plane_map"])+.010+held["bottom_below_palm_m"]-mt[2]-z0[2]
   if 0<needed<=.69:elbow=flex;break
  record(dict(event="placement_elbow_selected",elbow=elbow,needed_lift_m=needed,positions=len(positions),support_max_m=float(support.distance_m.max()),required_m=radius+.03))
  zero=[0,elbow,0,-math.pi-elbow,0,holding];pz,rz=forward(zero);contact0=pz+rz@np.array([0,.00005,.062125866]);body0=contact0.copy();body0[:2]+=held['center_minus_contact_xy']
  for xy in positions:
   h=float(np.r_[xy,1.]@plane['plane_map']);palmheight=h+.010+held['bottom_below_palm_m']-mt[2];lift=palmheight-pz[2]
   if not 0<lift<=.69:continue
   predicted=map_point(s,np.r_[body0[:2],h]);delta=(mr.T@np.r_[xy-predicted[:2],0.])[:2]
   if delta[0]<0 or not base_path(st['poses']['map'],[*delta,0.],table)['ready']:continue
   choices.append((float(np.linalg.norm(delta)),xy,lift,delta))
  choices.sort(key=lambda v:v[0]);selected=None
  for dist,xy,lift,delta in choices:
   patch=clear_patch(points,xy,np.array(plane['plane_map']),radius+.01)
   if patch['clear']:selected=(xy,lift,delta,patch);break
  if selected is None:raise ValueError('no_clear_reachable_patch_for_whole_controller')
  xy,lift,delta,patch=selected;plan=dict(release_center_map=xy.tolist(),release_lift_m=float(lift),delta=delta.tolist(),radius_m=radius,table_plane=plane,patch=patch,shape_source='grasp_anchored_geometry_with_preserved_grip');write_json(root/'placement-plan.json',plan)
  stage('prepare_downward_outside_table');goal=st['joints'].copy();goal[0]=.69;r.move_joints(goal,'raise_held_outside_table');goal=[.69,elbow,0,-math.pi-elbow,0,holding];r.move_joints(goal,'restore_downward_held_pose')
  stage('approach_measured_empty_patch')
  for i in range(4):
   item=r.sample();ss=item['snapshot'];held_image(item['hand']);palm=ss['base_from_palm_at_depth'];body=np.array(palm['translation'])+rotation(palm['quaternion'])@np.array([0,.00005,.062125866]);body[:2]+=held['center_minus_contact_xy'];mapped=map_point(ss,body);shift=rotation(ss['tracking_transforms']['map_from_base_at_depth']['quaternion']).T@np.r_[xy-mapped[:2],0.]
   if np.linalg.norm(shift[:2])<.012:break
   st=r.state();backend.stage='approach_placement';backend.adjust('place-approach-'+str(i),st['poses']['map'],[float(shift[0]),float(shift[1]),0.],st['joints'][:5],holding,single_placement_approach=True)
  stage('lower_to_table');goal=r.state()['joints'].copy();goal[0]=float(lift);r.move_joints(goal,'lower_to_measured_dining_height')
  item=r.sample();ss=item['snapshot'];proof=held_image(item['hand']);palm=ss['base_from_palm_at_depth'];body=np.array(palm['translation'])+rotation(palm['quaternion'])@np.array([0,.00005,.062125866]);body[:2]+=held['center_minus_contact_xy'];mapped=map_point(ss,body);check=support.query(mapped[:2],radius)
  if not check['safe']:raise ValueError('actual_held_footprint_not_over_support')
  r.save(item,'pre-release');write_json(root/'actual-release-support.json',dict(edge=check,held_texture=proof,center_map=mapped.tolist(),pose_is_kinematic=True));stage('release');goal=r.state()['joints'].copy();goal[5]=.65;r.move_joints(goal,'release_controller_on_dining',hand=True);report['release_commands_sent']=1
  stage('clear_downward_fingers');goal=r.state()['joints'].copy();goal[0]=.69;r.move_joints(goal,'raise_open_to_lift_limit');goal[1]=min(elbow+.15,-1.25);goal[3]=-math.pi-goal[1];r.move_joints(goal,'raise_downward_fingers_using_elbow');st=r.state();backend.stage='withdraw';backend.adjust('empty-retreat',st['poses']['map'],[-.4,0,0],st['joints'][:5],st['joints'][5]);r.prepare_navigation_from_current();r.face_source();r.save(r.sample(),'placed-controller');report.update(status='controller_released_placement_review_pending',last_state=r.state())
 except BaseException as e:report.update(status='stopped_requires_review',reason=str(e));record(dict(event='error',reason=str(e)))
 finally:
  if backend:backend.close()
  if r:r.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
