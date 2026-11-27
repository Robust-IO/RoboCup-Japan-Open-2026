"""Read-only independent laser/map check. Never publishes TF or robot commands."""
from competition_paths import is_run_path
import argparse,hashlib,json,math,time
from collections import deque
from pathlib import Path
import numpy as np,yaml
from PIL import Image
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener
from apple_transfer_automation import RobotLock,write_json
from hand_camera_ray import rotation
from match_reviewed_room_scan import search

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
if not is_run_path(a.output.resolve()):raise ValueError('D_output_required')
def td(t):
 t=t.transform;return dict(translation=[t.translation.x,t.translation.y,t.translation.z],quaternion=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
with RobotLock():
 a.output.mkdir();rclpy.init();n=rclpy.create_node('source_laser_map_readonly_audit');buf=Buffer();listener=TransformListener(buf,n);queue=deque(maxlen=40);rows=[];seen=set();report=dict(robot_commands_sent=False,map_transform_changed=False)
 sub=n.create_subscription(LaserScan,'/hsrb/base_scan',queue.append,qos_profile_sensor_data)
 try:
  deadline=time.monotonic()+10
  while time.monotonic()<deadline and len(rows)<3:
   rclpy.spin_once(n,timeout_sec=.02)
   for scan in list(queue):
    when=Time.from_msg(scan.header.stamp);stamp=when.nanoseconds
    if stamp in seen or not 0<=(n.get_clock().now().nanoseconds-stamp)/1e9<.4 or (rows and stamp-rows[-1]['stamp_ns']<100_000_000):continue
    pairs=[('base_from_scan','base_footprint',scan.header.frame_id),('odom_from_base','odom','base_footprint'),('map_from_base','map','base_footprint')]
    if not all(buf.can_transform(t,s,when) for _,t,s in pairs):continue
    row=dict(stamp_ns=stamp,angle_min=scan.angle_min,angle_increment=scan.angle_increment,ranges=[float(v) if np.isfinite(v) else None for v in scan.ranges])
    row.update({key:td(buf.lookup_transform(t,s,when)) for key,t,s in pairs});rows.append(row);seen.add(stamp);break
  if len(rows)<3:raise ValueError('three_fresh_scan_time_transforms_not_available')
  write_json(a.output/'scans.json',rows)
  paths=[Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/src/handyman_rebuild_ros2/maps/LayoutA/map.yaml'),b.parent/'handyman-export/LayoutA/map.yaml']
  mapyaml=next(q for q in paths if q.exists());meta=yaml.safe_load(mapyaml.read_text());pgm=mapyaml.parent/meta['image'];grid=np.asarray(Image.open(pgm));origin=meta['origin'];res=meta['resolution'];results=[]
  for row in rows:
   ranges=np.array([float('nan') if x is None else x for x in row['ranges']]);idx=np.flatnonzero(np.isfinite(ranges)&(ranges>.15)&(ranges<9))[::3]
   angles=row['angle_min']+idx*row['angle_increment'];pts=np.c_[ranges[idx]*np.cos(angles),ranges[idx]*np.sin(angles),np.zeros(len(idx))];tf=row['base_from_scan'];pts=(pts@rotation(tf['quaternion']).T+tf['translation'])[:,:2]
   tf=row['map_from_base'];r=rotation(tf['quaternion']);pose=[*tf['translation'][:2],math.atan2(r[1,0],r[0,0])]
   original=search(pts,grid,origin,res,[(v,v) for v in pose],[1,1,1])[0]
   coarse=search(pts,grid,origin,res,[(pose[0]-.65,pose[0]+.65),(pose[1]-.65,pose[1]+.65),(pose[2]-.2,pose[2]+.2)],[.05,.05,.025]);best=coarse[0][1]
   fine=search(pts,grid,origin,res,[(best[0]-.06,best[0]+.06),(best[1]-.06,best[1]+.06),(best[2]-.025,best[2]+.025)],[.005,.005,.0025])[0]
   results.append(dict(stamp_ns=row['stamp_ns'],estimated_pose=pose,current_score=original[0],current_inlier_fraction=original[2],matched_pose=fine[1],matched_score=fine[0],matched_inlier_fraction=fine[2],correction_xy_yaw=(np.asarray(fine[1])-pose).tolist()))
  report.update(status='readonly_scan_map_comparison_completed',map=str(pgm),map_sha256=hashlib.sha256(pgm.read_bytes()).hexdigest(),resolution=res,results=results,
   limitation='Scan matching is an independent estimate, not Unity ground truth. No correction applied.')
 except Exception as exc:report.update(status='readonly_audit_incomplete',reason=str(exc))
 finally:n.destroy_node();rclpy.shutdown();write_json(a.output/'result.json',report);print(json.dumps(report))
