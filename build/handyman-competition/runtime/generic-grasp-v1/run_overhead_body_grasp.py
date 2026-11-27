"""Measured central-body top-down pinch; class is only the detection identity."""
import sys,json,time,uuid,os,argparse,math
from pathlib import Path
import numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from apple_worker_client import Worker
from stopped_capture_runtime import with_stopped_capture
from horizontal_can_trial import HorizontalCanRuntime
from horizontal_can_grasp_plan import forward
from local_handle_geometry import source_points,plane_base,register_surface
from thin_box_clearance import ThinBoxClearance
from hand_camera_ray import rotation
from canned_grasp_geometry import transform_point
from can_destination_surface import measure as measure_table
p=argparse.ArgumentParser();p.add_argument('--target',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=a.output
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='white_side_table#kitchen');contact_local=np.array([0,.00005,.062125866])
class BodyRuntime(HorizontalCanRuntime):
 def contact_error(self,center,snapshot):
  palm=self.palm_pose(snapshot);return np.array(center)-(np.asarray(palm['translation'])+rotation(palm['quaternion'])@contact_local)
 def is_aligned(self,center,snapshot):return bool((abs(self.contact_error(center,snapshot))<=np.array([.006,.012,.006])).all())
report=dict(status='starting',target=a.target,strategy='top_down_central_solid_body',close_commands_sent=0,release_commands_sent=0,grasp_success_verified=False);runtime=worker=None
with RobotLock():
 root.mkdir();log=(root/'journal.jsonl').open('w')
 def record(e):
  if e.get('event')=='can_closure_sent':report['close_commands_sent']+=1
  log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
 def stage(v):report['stage']=v;record(dict(event='stage',stage=v))
 try:
  import rclpy
  rclpy.init();runtime=with_stopped_capture(BodyRuntime)(root,'body-overhead-'+uuid.uuid4().hex,record);stage('current_target_measurement');item=runtime.sample();s=item['snapshot'];cap=runtime.save(item,'source');os.environ['HANDYMAN_RESIDENT_MODEL']='1';worker=Worker(cfg['model_python'],cfg['registry'],root/'model.log',imgsz=960);raw=worker.infer(cap,pump=lambda:runtime.pump(0));worker.close();worker=None;write_json(root/'detections.json',raw)
  ds=[d for d in raw['detections'] if d['name']==a.target and d['confidence']>.8]
  if len(ds)!=1:raise ValueError('unique_current_target_required')
  plane=measure_table(item['depth'],s,table);bc=plane_base(s,plane['plane_map']);pts=source_points(item['depth'],s,ds[0]['box'],bc);y=float(np.median(pts[:,1]));band=pts[abs(pts[:,1]-y)<.015];xmin,xmax=np.quantile(band[:,0],[.02,.98]);x=float((xmin+xmax)/2);top=float(np.quantile(band[:,2],.98));bottom=float(np.r_[x,y,1.]@bc);z=top-.015
  if not .02<xmax-xmin<.083 or len(band)<100:raise ValueError('central_body_not_between_open_fingers')
  center=np.array([x,y,z]);zero=[0.,-math.pi/2,0.,-math.pi/2,-math.pi/2,.65];pal,rot=forward(zero);pzero=pal+rot@contact_local;lift=float(z+.10-pzero[2]);goal=[lift,*zero[1:]];forward(goal)
  # Verify downwards fingers, front/back closure, and complete collider bottom clearance.
  if np.linalg.norm(rot[:,2]-[0,0,-1])>.01 or abs(rot[0,1])<.99:raise ValueError('top_down_front_back_orientation_required')
  safety=ThinBoxClearance();palm_target=center-rot@contact_local;lowest=1e9
  for sample in safety.envelope['samples']:
   for part in sample['parts']:
    if part.get('is_trigger'):continue
    v=np.asarray(part['vertices_palm_m'])@rot.T+palm_target;clear=v[:,2]-(v[:,:2]@bc[:2]+bc[2]);lowest=min(lowest,float(clear.min())-part['contact_offset_m']-safety.envelope['environment_contact_offset_m']-safety.envelope['uncertainty_bound_m'])
  if lowest<.005:raise ValueError('downward_fingers_would_contact_table')
  write_json(root/'plan.json',dict(center=center.tolist(),central_body_width_m=float(xmax-xmin),top_m=top,table_height_m=bottom,closing_sweep_table_clearance_m=lowest,overhead_goal=goal,contact_local=contact_local.tolist()))
  tf=s['tracking_transforms']['odom_from_base_at_depth'];r=rotation(tf['quaternion']);t=np.asarray(tf['translation']);template=pts[(abs(pts[:,1]-y)>.035)&(pts[:,2]>pts[:,:2]@bc[:2]+bc[2]+.022)]@r.T+t;anchor=transform_point(s,center);serial=0
  def observe():
   global template,anchor,serial
   item=runtime.sample();ss=item['snapshot']
   try:m,shift=register_surface(item['depth'],ss,template,anchor)
   except Exception:
    runtime.save(item,'rejected-'+str(serial));raise
   template+=shift;anchor+=shift;runtime.fresh_measurement(ss);serial+=1;runtime.save(item,'observed-'+str(serial),m);return item,m
  stage('prepare_above_table');target=runtime.state()['joints'].copy();target[0]=goal[0];runtime.move_joints(target,'raise_tucked');runtime.move_joints(goal,'orient_downward_front_back');runtime.face_source((0,-.8));stage('align_above_solid_body')
  for i in range(8):
   item,m=observe();err=runtime.contact_error(m['center_m'],item['snapshot'])
   if abs(err[0])<.006 and abs(err[1])<.012:break
   runtime.base_move([float(err[0]),float(err[1]),0.],runtime.state()['joints'][:5],'align-overhead-'+str(i))
  else:raise ValueError('overhead_alignment_unresolved')
  stage('descend');item,m=observe();err=runtime.contact_error(m['center_m'],item['snapshot']);target=runtime.state()['joints'].copy();target[0]+=float(err[2]);runtime.move_joints(target,'lower_to_actual_solid_body')
  stage('close');closure,before,before_m=runtime.close_gripper(observe,target[:5]);write_json(root/'close.json',closure);holding=closure['closure_evidence']['observed_opening_rad'];report['holding_opening_rad']=holding
  baseline,bm=observe();old=transform_point(baseline['snapshot'],bm['center_m']);oldpalm=transform_point(baseline['snapshot'],baseline['snapshot']['palm_base_at_depth']);ot=template.copy();oa=anchor.copy();stage('lift');target=runtime.state()['joints'].copy();target[0]+=.08;runtime.move_joints(target,'lift_held_object');rows=[]
  for i in range(3):
   item=runtime.sample(force_new=True);ss=item['snapshot'];delta=transform_point(ss,ss['palm_base_at_depth'])-oldpalm
   try:m,shift=register_surface(item['depth'],ss,ot+delta,oa+delta)
   except Exception:
    runtime.save(item,'lift-rejected-'+str(i));raise
   actual=transform_point(ss,m['center_m'])-old;runtime.save(item,'lift-proof-'+str(i),m)
   if actual[2]<.05 or np.linalg.norm(actual-delta)>.025:raise ValueError('actual_object_not_following_lift')
   rows.append(dict(object_delta=actual.tolist(),palm_delta=delta.tolist(),measurement=m));runtime.pump(.25)
  report.update(status='overhead_body_grasp_verified',grasp_success_verified=True,object_remains_held=True,lift_proof=rows,transport_performed=False)
 except BaseException as e:report.update(status='stopped_requires_current_state_review',reason=str(e));record(dict(event='error',reason=str(e)))
 finally:
  if worker:worker.close()
  if runtime:
   try:report['last_state']=runtime.state()
   except Exception:pass
   runtime.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
