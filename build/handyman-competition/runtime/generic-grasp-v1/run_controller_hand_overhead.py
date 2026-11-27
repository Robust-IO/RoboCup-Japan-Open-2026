"""Continue the prepared overhead trial using a reviewed actual hand-image anchor."""
import sys,json,time,uuid,math
from pathlib import Path
import cv2,numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from stopped_capture_runtime import with_stopped_capture
from horizontal_can_trial import HorizontalCanRuntime
from hand_camera_ray import rotation,HAND_RASTER_TO_FRAME
old=b/'generic-overhead-grasp-20261107-172251';plan=json.loads((old/'plan.json').read_text());reference=b/'controller-hand-check-20261107-172409/captures/paired/hand.png';im=cv2.imread(str(reference));anchor_pixel=np.array([347.,275.]);mask=np.zeros(im.shape[:2],np.uint8);cv2.fillPoly(mask,[np.array([[240,145],[278,145],[298,222],[385,222],[402,145],[441,145],[478,290],[460,320],[235,320],[216,280]])],255);sift=cv2.SIFT_create(nfeatures=1200);ka,da=sift.detectAndCompute(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),mask)
def fit(image):
 kb,db=sift.detectAndCompute(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),None)
 if db is None:raise ValueError('hand_texture_missing')
 matches=[m for m,n in cv2.BFMatcher().knnMatch(da,db,k=2) if m.distance<.72*n.distance]
 if len(matches)<8:raise ValueError('same_object_hand_matches_insufficient')
 aa=np.float32([ka[m.queryIdx].pt for m in matches]);bb=np.float32([kb[m.trainIdx].pt for m in matches]);H,good=cv2.findHomography(aa,bb,cv2.RANSAC,3.)
 if H is None or int(good.sum())<8:raise ValueError('same_object_hand_transform_missing')
 xy=cv2.perspectiveTransform(anchor_pixel.reshape(1,1,2).astype(np.float32),H)[0,0]
 if not (0<xy[0]<640 and 0<xy[1]<480):raise ValueError('body_anchor_outside_hand_view')
 return xy,dict(matches=len(matches),inliers=int(good.sum()),homography=H.tolist())
class R(HorizontalCanRuntime):
 def contact_error(self,center,s):
  p=self.palm_pose(s);return np.asarray(center)-(np.asarray(p['translation'])+rotation(p['quaternion'])@np.array([0,.00005,.062125866]))
 def is_aligned(self,center,s):return bool((abs(self.contact_error(center,s))<=[.008,.012,.006]).all())
root=b/('controller-hand-overhead-'+time.strftime('%Y%m%d-%H%M%S'));report=dict(status='starting',target='game_controller',close_commands_sent=0,release_commands_sent=0,grasp_success_verified=False,anchor_source='assistant_reviewed_actual_hand_image_solid_body',hand_has_depth_sensor=False);runtime=None
with RobotLock():
 root.mkdir();log=(root/'journal.jsonl').open('w');(b/'controller-hand-overhead-latest.json').write_text(json.dumps(dict(output=str(root))))
 def record(e):
  if e.get('event')=='can_closure_sent':report['close_commands_sent']+=1
  log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
 def stage(v):report['stage']=v;record(dict(event='stage',stage=v))
 try:
  import rclpy
  rclpy.init();runtime=with_stopped_capture(R,profile='hand')(root,'hand-body-'+uuid.uuid4().hex,record);runtime.sample();state=runtime.state()
  if max(abs(np.array(state['joints'][1:5])-[-math.pi/2,0,-math.pi/2,-math.pi/2]))>.03 or state['joints'][5]<.64:raise ValueError('prepared_open_downward_pose_required')
  serial=0
  def observe():
   global serial
   item=runtime.sample();s=item['snapshot']
   try:pixel,proof=fit(item['hand'])
   except Exception:
    runtime.save(item,'rejected-'+str(serial));raise
   ray=rotation(s['base_from_hand']['quaternion'])@HAND_RASTER_TO_FRAME@np.linalg.solve(np.array(s['hand_k']).reshape(3,3),[*pixel,1.]);origin=np.array(s['base_from_hand']['translation']);scale=(plan['top_m']-origin[2])/ray[2]
   if scale<=0:raise ValueError('invalid_known_surface_hand_ray')
   xyz=origin+scale*ray;xyz[2]=plan['center'][2]
   m=dict(center_m=xyz.tolist(),pixel=pixel.tolist(),stamp_ns=s['streams']['hand']['stamp_ns'],hand_texture=proof,depth_measured=False,height_source='prior_current_task_head_RGBD',position_source='current_hand_texture_ray_intersection_with_measured_body_height');runtime.fresh_measurement(s);serial+=1;runtime.save(item,'hand-'+str(serial),m);return item,m
  stage('hand_visual_alignment')
  for i in range(5):
   item,m=observe();error=runtime.contact_error(m['center_m'],item['snapshot'])
   if abs(error[0])<.006 and abs(error[1])<.008:break
   runtime.base_move([float(error[0]),float(error[1]),0],runtime.state()['joints'][:5],'hand-align-'+str(i))
  else:raise ValueError('hand_alignment_unresolved')
  stage('descend_to_solid_body');item,m=observe();err=runtime.contact_error(m['center_m'],item['snapshot']);goal=runtime.state()['joints'].copy();goal[0]+=float(err[2]);runtime.move_joints(goal,'descend_after_actual_hand_alignment')
  stage('close');closure,before,bm=runtime.close_gripper(observe,goal[:5]);holding=closure['closure_evidence']['observed_opening_rad'];report['holding_opening_rad']=holding;write_json(root/'closure.json',closure)
  baseline=runtime.sample(force_new=True);xy0,proof0=fit(baseline['hand']);H0=np.array(proof0['homography']);runtime.save(baseline,'before-lift');initial=runtime.state();stage('lift');goal=initial['joints'].copy();goal[0]+=.08;runtime.move_joints(goal,'lift_controller');proofs=[]
  for i in range(3):
   item=runtime.sample(force_new=True);xy,pr=fit(item['hand']);H=np.array(pr['homography'])@np.linalg.inv(H0);p0=xy0.reshape(1,1,2).astype(np.float32);pts=np.array([xy0,xy0+[10,0],xy0+[0,10]],np.float32).reshape(-1,1,2);mapped=cv2.perspectiveTransform(pts,H).reshape(-1,2);scales=[float(np.linalg.norm(mapped[j]-mapped[0])/10) for j in (1,2)];movement=float(np.linalg.norm(mapped[0]-xy0));lift=runtime.state()['joints'][0]-initial['joints'][0];runtime.save(item,'lift-proof-'+str(i))
   if lift<.07 or min(scales)<.88 or max(scales)>1.12 or movement>20:raise ValueError('object_texture_not_following_actual_wrist_lift')
   proofs.append(dict(actual_joint_lift_m=lift,same_object_texture_scale=scales,texture_motion_pixels=movement,matches=pr['inliers'],object_depth_measured=False));runtime.pump(.25)
  report.update(status='controller_grasp_lift_verified',grasp_success_verified=True,object_remains_held=True,lift_proof=proofs,transport_performed=False)
 except BaseException as e:report.update(status='stopped_requires_current_state_review',reason=str(e));record(dict(event='error',reason=str(e)))
 finally:
  if runtime:
   try:report['last_state']=runtime.state()
   except Exception:pass
   runtime.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
