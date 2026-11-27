"""Current RGBD/table/physical-envelope diagnosis only; sends no commands."""
import json,uuid
from pathlib import Path
import numpy as np
from flat_box_grasp import BoxRuntime
from flat_box_partial_top import observe_known
from thin_box_clearance import ThinBoxClearance,plane_in_map
from hand_camera_ray import rotation
from canned_grasp_geometry import transform_point
from can_destination_surface import measure as measure_table
from apple_transfer_auto_plan import map_point
from apple_transfer_automation import RobotLock,write_json
import rclpy
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');old=b/'cigarette-grasp-20261101-10';root=b/'cigarette-contact-diagnostic-20261101-01'
with RobotLock():
 root.mkdir(exist_ok=False);rclpy.init();runtime=BoxRuntime(root,'thin-diagnostic-'+uuid.uuid4().hex,lambda _:None)
 try:
  model=json.loads((old/'source-geometry.json').read_text());appearance=json.loads((old/'appearance.json').read_text());source=json.loads((old/'source-snapshot.json').read_text())
  last=sorted((old/'captures').glob('box-*'))[-1];m0=json.loads((last/'object-measurement.json').read_text());s0=json.loads((last/'snapshot.json').read_text())
  center=transform_point(s0,m0['center_m']);axis_odom=rotation(source['tracking_transforms']['odom_from_base_at_depth']['quaternion'])@model['axis_base']
  cfg=json.loads((runtime.code/'config/apple_transfer_auto_v1.json').read_text());table=next(x for x in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if x['id']=='white_side_table#bedroom_1')
  safety=ThinBoxClearance();sample=runtime.sample(processing_reserve_ns=0);s=sample['snapshot'];palm=runtime.palm_pose(s)
  expected=transform_point(s,center,inverse=True);axis=rotation(s['tracking_transforms']['odom_from_base_at_depth']['quaternion']).T@axis_odom
  m=observe_known(sample['rgb'],sample['depth'],s,expected,axis,model,appearance,allow_locked=True)
  original_plane=plane_in_map(source,model['support_plane_base']);current_plane=measure_table(sample['depth'],s,table)['plane_map'];height=float(map_point(s,np.asarray(palm['translation']))[2])
  rows=[]
  for label,plane in [('source_ring',original_plane),('current_table',current_plane)]:
   proof=safety.proposal(s,m['center_m'],model['height_m'],palm['translation'],rotation(palm['quaternion']),plane)
   rows.append(dict(label=label,plane_map=list(plane),actual_palm_height_map_m=height,raise_to_floor_m=proof['minimum_allowed_palm_height_m']-height,**proof))
  path=runtime.save(sample,'current',m)
  write_json(root/'result.json',dict(status='diagnostic_only',robot_commands_sent=False,state=runtime.state(),measurement=m,clearance=rows,capture=str(path)))
  print((root/'result.json').read_text())
 finally:runtime.shutdown();rclpy.shutdown()
