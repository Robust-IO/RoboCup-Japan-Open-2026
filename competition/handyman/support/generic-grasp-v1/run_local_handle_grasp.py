import sys,json,time,uuid,os,argparse
from pathlib import Path
import numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from apple_worker_client import Worker
from stopped_capture_runtime import with_stopped_capture
from pepper_runtime import PepperRuntime
from thin_box_clearance import ThinBoxClearance
from horizontal_can_grasp_plan import forward
from table_transfer_core import FLEX,WRIST,OFFSET
from local_handle_geometry import source_points,plane_base,select_pinch,register_surface
from hand_camera_ray import rotation
from canned_grasp_geometry import transform_point
from can_destination_surface import measure as measure_table
p=argparse.ArgumentParser();p.add_argument('--target',required=True);p.add_argument('--source-table',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=a.output
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==a.source_table)
report=dict(status='starting',target=a.target,strategy='measured_local_handle_side_pinch',close_commands_sent=0,grasp_success_verified=False,release_commands_sent=0);runtime=worker=None
class HandleRuntime(PepperRuntime):
 def contact_error(self,center,snapshot):
  palm=self.palm_pose(snapshot);return np.array(center)-(np.asarray(palm['translation'])+rotation(palm['quaternion'])@np.array([.0000000685,.00005,.062125866]))
 def is_aligned(self,center,snapshot):return bool((abs(self.contact_error(center,snapshot))<=np.array([.012,.006,.006])).all())
 def approach_plan(self,error):
  x,y,z=map(float,error)
  if abs(y)>.006:
   if x<.09:return dict(phase='clear_before_lateral_alignment',delta=[x-.10,0.,0.])
   return dict(phase='lateral_alignment',delta=[0.,y,0.])
  return dict(phase='coarse_approach_before_finger_entry' if x>.16 else 'straight_contact_approach',delta=[x-.12 if x>.16 else x,0.,0.])
with RobotLock():
 root.mkdir();log=(root/'journal.jsonl').open('w')
 def record(e):
  if e.get('event')=='can_closure_sent':report['close_commands_sent']+=1
  log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
 def stage(v):report['stage']=v;record(dict(event='stage',stage=v))
 try:
  import rclpy
  rclpy.init();runtime=with_stopped_capture(HandleRuntime)(root,'local-handle-'+uuid.uuid4().hex,record)
  stage('identify_and_select_measured_grasp');item=runtime.sample();s=item['snapshot'];cap=runtime.save(item,'source');os.environ['HANDYMAN_RESIDENT_MODEL']='1';worker=Worker(cfg['model_python'],cfg['registry'],root/'model.log',imgsz=960);raw=worker.infer(cap,pump=lambda:runtime.pump(0));worker.close();worker=None;write_json(root/'detections.json',raw)
  ds=[d for d in raw['detections'] if d['name']==a.target and d['confidence']>=.8]
  if len(ds)!=1:raise ValueError('unique_current_requested_target_required')
  plane=measure_table(item['depth'],s,table);bc=plane_base(s,plane['plane_map']);pts=source_points(item['depth'],s,ds[0]['box'],bc);safety=ThinBoxClearance();_,flat=forward([.25,FLEX,0,WRIST,0,.65]);chosen,candidates=select_pinch(pts,bc,safety.envelope,flat);write_json(root/'grasp-candidates.json',dict(chosen=chosen,candidates=candidates,table_plane=plane));runtime.fresh_measurement(s)
  tf=s['tracking_transforms']['odom_from_base_at_depth'];r=rotation(tf['quaternion']);t=np.array(tf['translation']);front=np.quantile(pts[:,0],.02)
  # Track the visible rear body, outside the fingers' selected front contact patch.
  template=pts[(pts[:,0]>front+.055)&(pts[:,2]>pts[:,:2]@bc[:2]+bc[2]+.022)]@r.T+t
  grasp=transform_point(s,chosen['center_m']);serial=0
  np.save(root/'source-surface-odom.npy',template)
  def observe():
   global template,grasp,serial
   item=runtime.sample();ss=item['snapshot']
   try:m,shift=register_surface(item['depth'],ss,template,grasp)
   except Exception:
    runtime.save(item,'rejected-'+str(serial));raise
   template+=shift;grasp+=shift;runtime.fresh_measurement(ss);serial+=1;runtime.save(item,'measured-'+str(serial),m);return item,m
  item,m=observe();stage('prepare_open_hand');arm=runtime.prepare(m['center_m'][2]-OFFSET,m['center_m'][2]-chosen['bottom_m']);runtime.face_source((0.,-.65))
  stage('align_then_approach')
  for i in range(10):
   item,m=observe()
   if runtime.is_aligned(m['center_m'],item['snapshot']):break
   runtime.approach(observe,arm,i)
  stage('close');closure,before,before_m=runtime.close_gripper(observe,arm);write_json(root/'close.json',closure);holding=closure['closure_evidence']['observed_opening_rad'];report['holding_opening_rad']=holding
  # Measure contact displacement before using it as the actual lift baseline.
  baseline,baseline_m=observe();old=transform_point(baseline['snapshot'],baseline_m['center_m']);oldpalm=transform_point(baseline['snapshot'],baseline['snapshot']['palm_base_at_depth']);original_template=template.copy();original_grasp=grasp.copy()
  stage('lift_and_verify');runtime.lift_holding(holding);rows=[]
  for i in range(3):
   item=runtime.sample(force_new=True);ss=item['snapshot'];palm=transform_point(ss,ss['palm_base_at_depth']);delta=palm-oldpalm
   try:m,shift=register_surface(item['depth'],ss,original_template+delta,original_grasp+delta)
   except Exception:
    runtime.save(item,'lift-rejected-'+str(i));raise
   now=transform_point(ss,m['center_m']);actual=now-old
   runtime.save(item,'lift-proof-'+str(i),m)
   if actual[2]<.06 or np.linalg.norm(actual-delta)>.025:raise ValueError('actual_handle_object_did_not_follow_lift')
   if abs(runtime.state()['joints'][5]-holding)>.03:raise ValueError('grip_changed')
   rows.append(dict(object_delta=actual.tolist(),palm_delta=delta.tolist(),measurement=m));runtime.pump(.25)
  report.update(status='local_handle_grasp_lift_verified',grasp_success_verified=True,object_remains_held=True,lift_proof=rows,transport_performed=False)
  write_json(root/'held-context.json',dict(target=a.target,holding_opening_rad=holding,chosen=chosen,snapshot=ss,measurement=m,grasp_success_verified=True))
  # User requested lift only: remain here holding, no destination navigation.
 except BaseException as e:report.update(status='stopped_requires_current_state_review',reason=str(e));record(dict(event='error',reason=str(e)))
 finally:
  if worker:worker.close()
  if runtime:
   try:report['last_state']=runtime.state()
   except Exception:pass
   runtime.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
