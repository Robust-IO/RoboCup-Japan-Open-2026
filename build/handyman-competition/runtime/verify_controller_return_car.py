import sys,json,time,uuid,os
from pathlib import Path
import numpy as np
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from stopped_capture_runtime import with_stopped_capture
from horizontal_can_trial import HorizontalCanRuntime
from outcome_transfer_runtime import OutcomeBackend
from apple_worker_client import Worker
from can_destination_surface import measure as measure_table
from local_handle_geometry import source_points,plane_base
from apple_transfer_auto_plan import map_point,footprint
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());T=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='dining_table#kitchen');root=b/('controller-verify-return-car-'+time.strftime('%Y%m%d-%H%M%S'));r=backend=worker=None;report=dict(status='starting')
with RobotLock():
 root.mkdir();write_json(b/'controller-verify-return-car-latest.json',dict(output=str(root)));log=(root/'journal.jsonl').open('w')
 def record(e):log.write(json.dumps(dict(time_ns=time.time_ns(),**e))+'\n');log.flush();write_json(root/'result.json',report)
 try:
  import rclpy
  rclpy.init();r=with_stopped_capture(HorizontalCanRuntime)(root,'verify-controller-'+uuid.uuid4().hex,record);os.environ['HANDYMAN_RESIDENT_MODEL']='1';worker=Worker(cfg['model_python'],cfg['registry'],root/'model.log',imgsz=960);rows=[]
  for i in range(3):
   item=r.sample(force_new=True);s=item['snapshot'];cap=r.save(item,'placed-'+str(i));raw=worker.infer(cap,pump=lambda:r.pump(0));write_json(cap/'detections.json',raw);ds=[d for d in raw['detections'] if d['name']=='game_controller' and d['confidence']>.1]
   if len(ds)!=1:raise ValueError('placed_controller_not_uniquely_identified')
   plane=measure_table(item['depth'],s,T);bc=plane_base(s,plane['plane_map']);pts=source_points(item['depth'],s,ds[0]['box'],bc);mapped=map_point(s,np.median(pts,axis=0));c,h=footprint(T);above=pts[:,2]-(pts[:,:2]@bc[:2]+bc[2]);height=float(np.quantile(above,.98));low=float(np.quantile(above,.02))
   if not (abs(mapped[:2]-c)<h-.05).all() or not .02<height<.14 or low>.035:raise ValueError('controller_not_supported_on_dining_table')
   rows.append(dict(identity_source='known_released_controller_and_assistant_visual_review_low_score_box_is_region_only',center_map=mapped.tolist(),height_above_table_m=height,lowest_visible_surface_above_table_m=low,confidence=ds[0]['confidence'],capture=str(cap)));r.pump(.2)
  centers=np.array([v['center_map'] for v in rows]);spread=float(np.max(np.linalg.norm(centers-centers.mean(0),axis=1)))
  if spread>.02 or r.state()['joints'][5]<.64:raise ValueError('placed_controller_not_stable')
  report.update(status='controller_placed_verified',placement_success_verified=True,rows=rows,spread_m=spread);write_json(root/'result.json',report)
  prev=Path(json.loads((b/'controller-dining-place-latest.json').read_text())['output']);value=json.loads((prev/'result.json').read_text());value.update(status='controller_placement_verified',placement_success_verified=True,verification=str(root));write_json(prev/'result.json',value)
  backend=OutcomeBackend(cfg);backend.bind(root,r.epoch,record);backend.observation_runtime=r;backend.stage='navigate_source';report['stage']='return_for_car';backend.navigate('kitchen','kitchen',[9.205,1.482962,-.012512],r.state()['joints'][5],'return-car-source');r.face_source();item=r.sample();cap=r.save(item,'car-source');raw=worker.infer(cap,pump=lambda:r.pump(0));write_json(cap/'detections.json',raw);write_json(b/'toy-car-source-latest.json',dict(output=str(root),capture=str(cap),detections=raw));report.update(status='returned_car_observed',car_detections=raw)
 except BaseException as e:report.update(status='stopped_requires_review',reason=str(e))
 finally:
  if worker:worker.close()
  if backend:backend.close()
  if r:r.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
