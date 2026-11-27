"""Measured round-body strategy over verified shared motion; target is an argument."""
import argparse,json,sys,time,uuid,os
from pathlib import Path
import numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from apple_worker_client import Worker
from stopped_capture_runtime import with_stopped_capture
from pepper_runtime import PepperRuntime
from canned_grasp_geometry import transform_point
from hand_camera_ray import rotation
from can_destination_surface import measure as measure_table
from round_body_geometry import measure
from table_transfer_core import OFFSET
p=argparse.ArgumentParser();p.add_argument('--resume-open',type=Path);p.add_argument('--target',required=True);p.add_argument('--source-table',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();root=a.output
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());table=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']==a.source_table)
report=dict(status='starting',target=a.target,strategy='measured_round_body_side_grasp',close_commands_sent=0,release_commands_sent=0,grasp_success_verified=False)
runtime=None;worker=None
with RobotLock():
 root.mkdir();log=(root/'journal.jsonl').open('w')
 def record(e):
  if e.get('event')=='can_closure_sent':report['close_commands_sent']+=1
  log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
 def stage(s):report['stage']=s;record(dict(event='stage',stage=s))
 try:
  import rclpy
  rclpy.init();R=with_stopped_capture(PepperRuntime,profile='rgbd');runtime=R(root,'generic-round-'+uuid.uuid4().hex,record)
  original_approach=runtime.approach_plan
  def approach_with_observation_standoff(error):
   plan=original_approach(error)
   if plan['phase']=='straight_contact_approach' and plan['delta'][0]>.16:
    plan=dict(plan,phase='coarse_approach_before_finger_entry',delta=[plan['delta'][0]-.12,0.,0.])
   return plan
  runtime.approach_plan=approach_with_observation_standoff
  os.environ['HANDYMAN_RESIDENT_MODEL']='1';worker=Worker(cfg['model_python'],cfg['registry'],root/'model.log',imgsz=960)
  if a.resume_open:
   original=json.loads((a.resume_open/'source-geometry.json').read_text());rows=original['rows'];ref=original['reference'];m=ref
   anchor=np.array(rows[-1]['center_odom']);serial=0;write_json(root/'source-geometry.json',original)
   worker.close();worker=None
  else:
   stage('identify_and_measure');rows=[]
   for i in range(3):
    sample=runtime.sample();s=sample['snapshot'];cap=runtime.save(sample,'identity-'+str(i));raw=worker.infer(cap,pump=lambda:runtime.pump(0.));write_json(cap/'detections.json',raw)
    targets=[d for d in raw['detections'] if d['name']==a.target and d['confidence']>=.8]
    if len(targets)!=1:raise ValueError('unique_requested_target_required')
    plane=measure_table(sample['depth'],s,table);c=plane['plane_map'];tf=s['tracking_transforms']['map_from_base_at_depth'];n=np.array([-c[0],-c[1],1.]);nb=rotation(tf['quaternion']).T@n;bc=np.r_[-nb[:2]/nb[2],(c[2]-n@tf['translation'])/nb[2]]
    m=measure(sample['rgb'],sample['depth'],s,box=targets[0]['box'],plane=bc);runtime.fresh_measurement(s);rows.append(dict(measurement=m,center_odom=transform_point(s,m['center_m']).tolist(),detection=targets[0]))
   worker.close();worker=None
   if max(np.linalg.norm(np.array(row['center_odom'])-rows[0]['center_odom']) for row in rows)>.015:raise ValueError('source_geometry_not_repeatable')
   ref=m;anchor=np.array(rows[-1]['center_odom']);serial=0;write_json(root/'source-geometry.json',dict(rows=rows,reference=ref))
  def observe():
   global anchor,serial
   item=runtime.sample();ss=item['snapshot'];expected=transform_point(ss,anchor,inverse=True)
   
   try:mm=measure(item['rgb'],item['depth'],ss,reference=ref,expected=expected)
   except Exception:
    runtime.save(item,'rejected-'+str(serial));raise
   runtime.fresh_measurement(ss)
   if np.linalg.norm(np.array(mm['center_m'])-expected)>.04:raise ValueError('target_surface_discontinuous')
   anchor=transform_point(ss,mm['center_m']);serial+=1;runtime.save(item,'measured-'+str(serial),mm);return item,mm
  lift=m['center_m'][2]-OFFSET
  if not 0<=lift<=.59:raise ValueError('physical_lift_or_verification_headroom_unavailable')
  stage('prepare')
  if a.resume_open:
   prior=json.loads((a.resume_open/'result.json').read_text());state=runtime.state()
   if prior['close_commands_sent']!=0 or abs(state['joints'][5]-.65)>.01:raise ValueError('unclosed_open_arm_required')
   arm=state['joints'][:5]
  else:arm=runtime.prepare(lift,m['center_m'][2]-m['bottom_z_m'])
  stage('approach')
  for i in range(10):
   item,m=observe()
   if runtime.is_aligned(m['center_m'],item['snapshot']):break
   runtime.approach(observe,arm,i)
  stage('close');close,before,before_m=runtime.close_gripper(observe,arm);write_json(root/'close.json',close)
  holding=close['closure_evidence']['observed_opening_rad'];old=transform_point(before['snapshot'],before_m['center_m']);oldpalm=transform_point(before['snapshot'],before['snapshot']['palm_base_at_depth'])
  stage('lift');runtime.lift_holding(holding);proof=[]
  for i in range(3):
   item=runtime.sample();s=item['snapshot'];palm=transform_point(s,s['palm_base_at_depth']);expected=transform_point(s,old+palm-oldpalm,inverse=True)
   m=measure(item['rgb'],item['depth'],s,reference=ref,expected=expected);now=transform_point(s,m['center_m']);delta=now-old
   if delta[2]<.06 or np.linalg.norm(delta-(palm-oldpalm))>.025:raise ValueError('actual_object_did_not_follow_lift')
   if abs(runtime.state()['joints'][5]-holding)>.03:raise ValueError('grip_changed_after_lift')
   cap=runtime.save(item,'lift-proof-'+str(i),m);proof.append(dict(capture=str(cap),object_delta=delta.tolist(),palm_delta=(palm-oldpalm).tolist()))
  report.update(grasp_success_verified=True,lift_proof=proof,holding_opening_rad=holding)
  write_json(root/'held-context.json',dict(target=a.target,reference=ref,measurement=m,snapshot=s,holding_opening_rad=holding,grasp_success_verified=True))
  stage('retract');runtime.retract_holding(holding);report.update(status='generic_round_body_grasp_verified',object_remains_held=True)
 except BaseException as e:report.update(status='stopped_requires_current_state_review',reason=str(e))
 finally:
  if worker:worker.close()
  if runtime:
   try:report['last_state']=runtime.state()
   except Exception:pass
   runtime.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
