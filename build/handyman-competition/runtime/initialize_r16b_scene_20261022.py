"""Use this scene's fresh laser/odom to validate configured initial localization.

No goal, velocity, arm, hand, head or competition protocol publication. No old
numeric map anchor is read. Navigation remains idle until the transfer entry.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener

code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,check_inputs,write_json
from hand_camera_ray import rotation
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'apple-r16b-live-initial-20261022-01'
cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
environment=yaml.safe_load(Path(cfg['environment']).read_text());initial=environment['initial_pose']
pose=np.array([initial['x'],initial['y'],initial['yaw']],float)
if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('reviewed_environment_required')
if (root/'localization-complete.json').exists() or (root/'initial-localizer.log').exists():raise ValueError('new_localization_attempt_required')
map_path=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/src/handyman_rebuild_ros2/maps/LayoutA/map.yaml')
meta=yaml.safe_load(map_path.read_text());grid=np.asarray(Image.open(map_path.parent/meta['image']))
if environment['internal_name']!='LayoutA' or meta['origin'][2]!=0 or meta.get('negate',0)!=0:raise ValueError('reviewed_map_format_required')
occupied=(255-grid.astype(float))/255>meta['occupied_thresh'];dist=distance_transform_edt(~occupied)*meta['resolution']
scans=json.loads((root/'unanchored-scans.json').read_text())['scans'];rows=[]
for scan in scans:
    ranges=np.array([np.nan if x is None else x for x in scan['ranges']])
    angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    idx=np.flatnonzero(np.isfinite(ranges)&(ranges>max(.15,scan['range_min']))&(ranges<min(9.,scan['range_max'])))[::3]
    points=np.c_[ranges[idx]*np.cos(angles[idx]),ranges[idx]*np.sin(angles[idx]),np.zeros(len(idx))]
    tf=scan['base_from_scan'];xy=(points@rotation(tf['quaternion']).T+tf['translation'])[:,:2]
    r=np.array([[math.cos(pose[2]),-math.sin(pose[2])],[math.sin(pose[2]),math.cos(pose[2])]])
    mapped=xy@r.T+pose[:2];cells=np.floor((mapped-meta['origin'][:2])/meta['resolution']).astype(int)
    u=cells[:,0];v=grid.shape[0]-1-cells[:,1];valid=(u>=0)&(u<grid.shape[1])&(v>=0)&(v<grid.shape[0])
    errors=np.full(len(u),10.);errors[valid]=dist[v[valid],u[valid]]
    row=dict(stamp_ns=scan['stamp_ns'],points=len(errors),inliers_within10cm=float(np.mean(errors<=.1)),median_error_m=float(np.median(errors)))
    rows.append(row)
audit=dict(configured_initial_pose=pose.tolist(),rows=rows,robot_commands_sent=False,old_numeric_anchor_reused=False)
write_json(root/'initial-laser-audit.json',audit)
if len(rows)!=3 or any(x['points']<60 or x['inliers_within10cm']<.9 or x['median_error_m']>.05 for x in rows):
    print(json.dumps(audit),flush=True);raise ValueError('current_scan_does_not_support_configured_initial_scene')

with RobotLock():
    inputs=check_inputs(cfg);write_json(root/'input-files-verified.json',inputs)
    rclpy.init();node=rclpy.create_node('verify_fresh_initial_anchor_context');buf=Buffer();listener=TransformListener(buf,node)
    try:
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        if buf.can_transform('map','odom',Time()):raise ValueError('existing_map_anchor_no_duplicate')
        tf=buf.lookup_transform('odom','base_footprint',Time());t=tf.transform.translation;q=tf.transform.rotation
        age=(node.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
        old=scans[-1]['odom_from_base'];old_r=rotation(old['quaternion']);yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        old_yaw=math.atan2(old_r[1,0],old_r[0,0])
        if not -.02<=age<=.4 or math.dist([t.x,t.y],old['translation'][:2])>.02 or abs(math.atan2(math.sin(yaw-old_yaw),math.cos(yaw-old_yaw)))>.03:
            raise ValueError('initial_robot_moved_since_current_laser_check')
    finally:node.destroy_node();rclpy.shutdown()
    command=['ros2','run','handyman_rebuild_ros2','handyman_initial_pose_localizer','--ros-args',
        '-p','map_frame:=map','-p','odom_frame:=odom','-p','base_frame:=base_footprint']
    for key in ('x','y','yaw'):command+=['-p','initial_pose.'+key+':='+str(float(initial[key]))]
    with (root/'initial-localizer.log').open('x') as stream:
        child=subprocess.Popen(command,env=dict(os.environ,ROS_LOG_DIR=str(root/'localizer-roslogs')),
            stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    startup=dict(pid=child.pid,command=command,current_laser_audit=audit,old_numeric_anchor_reused=False,robot_commands_sent=False)
    write_json(root/'initial-localizer-startup.json',startup)
    end=time.monotonic()+8
    while time.monotonic()<end:
        if child.poll() is not None:raise ValueError('new_initial_localizer_exited')
        if 'Aligned' in (root/'initial-localizer.log').read_text(errors='replace'):break
        time.sleep(.1)
    else:raise ValueError('initial_localizer_not_aligned')
    commands={
        'ready':[sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(root/'ready.json')],
        'cameras':[sys.executable,str(code/'capture_grasp_scene.py'),str(root/'mapped-cameras'),
            '--hand-geometry','--map-geometry','--tracking-epoch','r16b-initial-mapped-20261022-01','--duration','7','--interval','1']}
    children={}
    for name,cmd in commands.items():
        with (root/('mapped-'+name+'.log')).open('x') as stream:
            children[name]=subprocess.Popen(cmd,env=dict(os.environ,ROS_LOG_DIR=str(root/'mapped-roslogs')),
                stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    codes={name:p.wait(timeout=30) for name,p in children.items()}
    result=dict(status='initial_localization_and_idle_readiness_verified' if not any(codes.values()) else 'readiness_incomplete',
        child_exit_codes=codes,localizer_pid=child.pid,current_initial_pose=pose.tolist(),robot_commands_sent=False,
        current_laser_verified=True,old_numeric_anchor_reused=False)
    write_json(root/'localization-complete.json',result);print(json.dumps(result),flush=True)
    if any(codes.values()):raise SystemExit(1)
