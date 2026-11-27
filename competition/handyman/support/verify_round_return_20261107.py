import sys,json,time,uuid
from pathlib import Path
import numpy as np
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path[:0]=[str(b/'generic-grasp-v1'),str(b),str(f)]
from apple_transfer_automation import RobotLock,write_json
from stopped_capture_runtime import with_stopped_capture
from pepper_runtime import PepperRuntime
from outcome_transfer_runtime import OutcomeBackend
from round_body_geometry import measure
from hand_camera_ray import rotation
from apple_transfer_auto_plan import map_point,footprint
from can_destination_surface import measure as measure_table
from tabletop_support import build_support,TableSupport
from apple_transfer_auto_plan import depth_in_map
p=b/'round-place-return-20261107-171228';root=b/('round-placement-recheck-'+time.strftime('%Y%m%d-%H%M%S'));held=json.loads((b/'generic-round-grasp-20261107-170503/held-context.json').read_text());ref=held['reference'];old=json.loads((p/'captures/placed-4/snapshot.json').read_text());om=json.loads((p/'captures/placed-4/object-measurement.json').read_text());expected=map_point(old,np.array(om['center_m']));cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());T=next(t for t in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if t['id']=='dining_table#kitchen');report=dict(status='starting');r=backend=None
with RobotLock():
 root.mkdir();log=(root/'journal.jsonl').open('w')
 def record(e):log.write(json.dumps(e)+'\n');log.flush()
 try:
  import rclpy
  rclpy.init();r=with_stopped_capture(PepperRuntime)(root,'cup-recheck-'+uuid.uuid4().hex,record);rows=[]
  for i in range(3):
   item=r.sample();s=item['snapshot'];tf=s['tracking_transforms']['map_from_base_at_depth'];e=rotation(tf['quaternion']).T@(expected-tf['translation']);m=measure(item['rgb'],item['depth'],s,reference=ref,expected=e);plane=measure_table(item['depth'],s,T);c=map_point(s,np.array(m['center_m']));bottom=map_point(s,np.r_[m['center_m'][:2],m['bottom_z_m']]);gap=float(bottom[2]-np.r_[bottom[:2],1.]@plane['plane_map']);bounds=np.minimum(c[:2]-plane['observed_xy_min'],np.array(plane['observed_xy_max'])-c[:2]);margin=float(min(bounds)-m['max_radius_m']);points=depth_in_map(item['depth'],s);dz=points[:,2]-np.c_[points[:,:2],np.ones(len(points))]@plane['plane_map'];dist=np.linalg.norm(points[:,:2]-c[:2],axis=1);ring=points[(abs(dz)<.006)&(dist>m['max_radius_m']+.005)&(dist<m['max_radius_m']+.045)];r.save(item,'placed-'+str(i),m)
   if abs(gap)>.015 or margin<.04 or len(ring)<100:raise ValueError('current_support_evidence_insufficient')
   rows.append(dict(center=c.tolist(),bottom_gap_m=gap,current_observed_edge_margin_m=margin,surrounding_table_points=len(ring),stamp_ns=m['stamp_ns']));r.pump(.25)
  if np.max(np.linalg.norm(np.array([v['center'] for v in rows])-np.mean([v['center'] for v in rows],axis=0),axis=1))>.015:raise ValueError('not_stable')
  report.update(status='cup_placement_verified',placement_success_verified=True,rows=rows,verification='current_table_plane_and_surrounding_support_plus_stable_actual_cup');write_json(root/'result.json',report)
  previous=json.loads((p/'result.json').read_text());previous.update(placement_success_verified=True,placement_recheck=str(root));write_json(p/'result.json',previous)
  backend=OutcomeBackend(cfg);backend.bind(root,r.epoch,record);backend.observation_runtime=r;backend.stage='navigate_source';backend.navigate('kitchen','kitchen',[9.205,1.482962,-.012512],r.state()['joints'][5],'return-white-table');r.face_source();item=r.sample();r.save(item,'controller-source');report.update(status='cup_placed_returned_for_controller');write_json(b/'controller-source-latest.json',dict(output=str(root),capture=str(root/'captures/controller-source')))
 except BaseException as e:report.update(status='stopped_requires_review',reason=str(e))
 finally:
  if backend:backend.close()
  if r:r.shutdown();rclpy.shutdown()
  write_json(root/'result.json',report);log.close();print(json.dumps(report))
