"""Freshly observe the requested box, translate laterally, then reobserve."""
from competition_paths import is_run_path
import argparse,json,os,time,uuid
from pathlib import Path
import numpy as np
from apple_transfer_automation import RobotLock,write_json
from flat_box_grasp import BoxRuntime,FINGER_TRIGGER_LOCAL
from flat_box_geometry import estimate
from apple_worker_client import Worker
from apple_transfer_auto_plan import map_point,footprint,depth_in_map,map_pose
from horizontal_can_grasp_plan import forward
from inverse_table_place_plan import FLEX,WRIST
from table_approach_reachability import base_path
from can_destination_surface import measure as measure_table

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
if os.environ.get('ROS_DOMAIN_ID')!='71' or not is_run_path(a.output.resolve()):raise ValueError('isolated_D_required')
cfg=json.loads(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1/config/apple_transfer_auto_v1.json').read_text());table=next(x for x in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if x['id']=='white_side_table#bedroom_1')
report=dict(status='starting',close_commands_sent=0,release_commands_sent=0);runtime=worker=None
with RobotLock():
 a.output.mkdir();import rclpy
 rclpy.init()
 try:
  runtime=BoxRuntime(a.output,'cigarette-lateral-'+uuid.uuid4().hex,lambda e:None);worker=Worker(cfg['model_python'],cfg['registry'],a.output/'model.json',imgsz=640)
  def observe(name):
   s=runtime.sample(processing_reserve_ns=0);path=runtime.save(s,name);ds=worker.infer(path,pump=runtime.pump);write_json(path/'detections.json',ds);ms=[];c,h=footprint(table)
   for d in ds['detections']:
    if d['name']!='cigarette':continue
    m=estimate(s['rgb'],s['depth'],s['snapshot'],d);mapped=map_point(s['snapshot'],np.asarray(m['center_m']))
    if (abs(mapped[:2]-c)<=h).all():ms.append(m)
   if not ms:raise ValueError('no_current_measured_cigarette_on_white_table')
   ms.sort(key=lambda m:float(np.linalg.norm(m['center_m'][:2])));return s,ms[0],path
  sample,m,path=observe('before-right-shift');snap=sample['snapshot'];state=runtime.state();runtime.ownership()
  if abs(state['joints'][5]-.65)>.005:raise ValueError('empty_open_hand_required')
  palm,r=forward([.25,FLEX,0,WRIST,0,.65]);contact=palm+r@FINGER_TRIGGER_LOCAL;delta=[0.,float(m['center_m'][1]-contact[1]),0.]
  reach=base_path(state['poses']['map'],delta,table)
  if not reach['ready']:raise ValueError('lateral_path_intersects_table')
  report.update(initial_state=state,measurement=m,lateral_delta_m=delta,reachability=reach);write_json(a.output/'result.json',report)
  runtime.fresh_measurement(snap);runtime.base_move(delta,state['joints'][:5],'move_right_before_straight_grasp')
  after,measured,capture=observe('after-right-shift');s=after['snapshot'];plane=measure_table(after['depth'],s,table);points=depth_in_map(after['depth'],s)
  coef=np.asarray(plane['plane_map']);near=abs(points[:,2]-(points[:,:2]@coef[:2]+coef[2]))<.004
  c,h=footprint(table);area=(abs(points[:,:2]-c)<h+.5).all(1);actual=points[near&area]
  report.update(status='right_shift_completed_and_target_reobserved',final_state=runtime.state(),measurement_after=measured,capture_after=str(capture),table_plane=plane,
   actual_visible_plane_bounds_map_m=[actual.min(0).tolist(),actual.max(0).tolist()] if len(actual) else None)
 except Exception as exc:report.update(status='stopped_requires_state_review',reason=str(exc))
 finally:
  if worker:worker.close()
  if runtime:runtime.shutdown()
  rclpy.shutdown();write_json(a.output/'result.json',report);print(json.dumps(report))
