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
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());T=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='dining_table#kitchen');held=json.loads((b/'controller-overhead-placement-context.json').read_text());holding=held['holding_opening_rad'];root=b/('controller-dining-place-'+time.strftime('%Y%m%d-%H%M%S'));report=dict(status='starting',target='game_controller',destination=T['id'],release_commands_sent=0,placement_success_verified=False);r=backend=None
ref=cv2.imread(str(b/'controller-overhead-handle-clearance-20261107-173547/captures/lift-proof-2/hand.png'));mask=np.zeros(ref.shape[:2],np.uint8);mask[80:410,420:640]=255;sift=cv2.SIFT_create(nfeatures=800);ka,da=sift.detectAndCompute(cv2.cvtColor(ref,cv2.COLOR_BGR2GRAY),mask)
def held_image(im):
 kb,db=sift.detectAndCompute(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),None)
 if db is None:raise ValueError('held_controller_texture_missing')
 matches=[m for m,n in cv2.BFMatcher().knnMatch(da,db,k=2) if m.distance<.75*n.distance]
 if len(matches)<8:raise ValueError('held_controller_texture_missing')
 aa=np.float32([ka[m.queryIdx].pt for m in matches]);bb=np.float32([kb[m.trainIdx].pt for m in matches]);H,ins=cv2.findHomography(aa,bb,cv2.RANSAC,3.)
 if H is None or ins.sum()<6:raise ValueError('held_controller_texture_unconfirmed')
 good=aa[ins.ravel()>0]
 if np.max(np.ptp(good,axis=0))<30:raise ValueError('object_texture_matches_too_local')
 return dict(inliers=int(ins.sum()),homography=H.tolist(),same_object_texture_seen=True,rigid_pose_unchanged_claimed=False)

with RobotLock():
 root.mkdir();log=(root/'journal.jsonl').open('w');write_json(b/'controller-dining-place-latest.json',dict(output=str(root)))
 def record(e):log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
 def stage(v):report['stage']=v;record(dict(event='stage',stage=v))
 try:
  import rclpy
  rclpy.init();r=with_stopped_capture(HorizontalCanRuntime,profile='paired')(root,'controller-place-'+uuid.uuid4().hex,record);backend=OutcomeBackend(cfg);backend.bind(root,r.epoch,record);backend.observation_runtime=r
  item=r.sample();s=item['snapshot'];st=r.state()
  if abs(st['joints'][5]-holding)>.03:raise ValueError('grip_changed')
  record(dict(event='actual_held_texture',proof=held_image(item['hand'])));r.save(item,'destination-table-held');stage('measure_clear_destination_patch');plane=measure_table(item['depth'],s,T);table=r.table_geometry(item,T,plane);center,half=footprint(table);points=depth_in_map(item['depth'],s);support=TableSupport(build_support(points,center,half,plane['plane_map'],T['id'],s['streams']['depth']['stamp_ns']),T['id']);radius=held['radius_m'];positions=support.safe_centers(radius,.02);choices=[];mr=rotation(s['tracking_transforms']['map_from_base_at_depth']['quaternion']);mt=np.array(s['tracking_transforms']['map_from_base_at_depth']['translation'])
  zero=[0,-math.pi/2,0,-math.pi/2,0,holding];pz,rz=forward(zero);contact0=pz+rz@np.array([0,.00005,.062125866]);body0=contact0.copy();body0[:2]+=held['center_minus_contact_xy']
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
  xy,lift,delta,patch=selected;plan=dict(release_center_map=xy.tolist(),release_lift_m=float(lift),delta=delta.tolist(),radius_m=radius,table_plane=plane,patch=patch,shape_source='grasp_anchored_geometry_with_current_hand_texture');write_json(root/'placement-plan.json',plan)
  stage('prepare_downward_outside_table');goal=st['joints'].copy();goal[0]=.69;r.move_joints(goal,'raise_held_outside_table');goal=[.69,-math.pi/2,0,-math.pi/2,0,holding];r.move_joints(goal,'restore_downward_held_pose')
  stage('approach_measured_empty_patch')
  for i in range(4):
   item=r.sample();ss=item['snapshot'];held_image(item['hand']);palm=ss['base_from_palm_at_depth'];body=np.array(palm['translation'])+rotation(palm['quaternion'])@np.array([0,.00005,.062125866]);body[:2]+=held['center_minus_contact_xy'];mapped=map_point(ss,body);shift=rotation(ss['tracking_transforms']['map_from_base_at_depth']['quaternion']).T@np.r_[xy-mapped[:2],0.]
   if np.linalg.norm(shift[:2])<.012:break
   st=r.state();backend.stage='approach_placement';backend.adjust('place-approach-'+str(i),st['poses']['map'],[float(shift[0]),float(shift[1]),0.],st['joints'][:5],holding,single_placement_approach=True)
  stage('lower_to_table');goal=r.state()['joints'].copy();goal[0]=float(lift);r.move_joints(goal,'lower_to_measured_dining_height')
  item=r.sample();ss=item['snapshot'];proof=held_image(item['hand']);palm=ss['base_from_palm_at_depth'];body=np.array(palm['translation'])+rotation(palm['quaternion'])@np.array([0,.00005,.062125866]);body[:2]+=held['center_minus_contact_xy'];mapped=map_point(ss,body);check=support.query(mapped[:2],radius)
  if not check['safe']:raise ValueError('actual_held_footprint_not_over_support')
  r.save(item,'pre-release');write_json(root/'actual-release-support.json',dict(edge=check,held_texture=proof,center_map=mapped.tolist(),pose_is_kinematic=True));stage('release');goal=r.state()['joints'].copy();goal[5]=.65;r.move_joints(goal,'release_controller_on_dining',hand=True);report['release_commands_sent']=1
  stage('clear_downward_fingers');goal=r.state()['joints'].copy();goal[0]=.69;r.move_joints(goal,'raise_open_to_lift_limit');goal[1]=-1.25;goal[3]=-math.pi+1.25;r.move_joints(goal,'raise_downward_fingers_using_elbow');st=r.state();backend.stage='withdraw';backend.adjust('empty-retreat',st['poses']['map'],[-.4,0,0],st['joints'][:5],st['joints'][5]);r.prepare_navigation_from_current();r.face_source();r.save(r.sample(),'placed-controller');report.update(status='controller_released_placement_review_pending',last_state=r.state())
 except BaseException as e:report.update(status='stopped_requires_review',reason=str(e));record(dict(event='error',reason=str(e)))
 finally:
  if backend:backend.close()
  if r:r.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
