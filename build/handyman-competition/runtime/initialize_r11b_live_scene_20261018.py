"""New initial-scene map anchor after independently checked fresh laser samples.

No robot motion/goal publication. Replaces ONLY the exact inspected old idle
localizer process group, then checks new laser/TF/readiness evidence.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,check_inputs,write_json
from hand_camera_ray import rotation

if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise ValueError('reviewed_simulation_environment_required')
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
out=base/'apple-r11b-initial-readiness-20261018-01'
cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
env=yaml.safe_load(Path(cfg['environment']).read_text());initial=env['initial_pose']
pose=np.array([initial['x'],initial['y'],initial['yaw']],float)
map_path=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/src/handyman_rebuild_ros2/maps/LayoutA/map.yaml')
meta=yaml.safe_load(map_path.read_text());grid=np.asarray(Image.open(map_path.parent/meta['image']))
if env['internal_name']!='LayoutA' or meta['origin'][2]!=0 or meta.get('negate',0)!=0:
    raise ValueError('unsupported_scene_or_map_format')
occupied=(255-grid.astype(float))/255>meta['occupied_thresh']
dist=distance_transform_edt(~occupied)*meta['resolution']

def score(path):
    scans=json.loads(path.read_text())['scans'];rows=[]
    for scan in scans:
        ranges=np.array([np.nan if v is None else v for v in scan['ranges']])
        angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
        idx=np.flatnonzero(np.isfinite(ranges)&(ranges>max(.15,scan['range_min']))&(ranges<min(9.,scan['range_max'])))[::3]
        points=np.c_[ranges[idx]*np.cos(angles[idx]),ranges[idx]*np.sin(angles[idx]),np.zeros(len(idx))]
        tf=scan['base_from_scan'];xy=(points@rotation(tf['quaternion']).T+tf['translation'])[:,:2]
        r=np.array([[math.cos(pose[2]),-math.sin(pose[2])],[math.sin(pose[2]),math.cos(pose[2])]])
        mapped=xy@r.T+pose[:2]
        cells=np.floor((mapped-meta['origin'][:2])/meta['resolution']).astype(int)
        u=cells[:,0];v=grid.shape[0]-1-cells[:,1]
        valid=(u>=0)&(u<grid.shape[1])&(v>=0)&(v<grid.shape[0])
        errors=np.full(len(u),10.);errors[valid]=dist[v[valid],u[valid]]
        row=dict(stamp_ns=scan['stamp_ns'],points=len(errors),
            endpoint_inlier_fraction_within10cm=float(np.mean(errors<=.1)),median_endpoint_error_m=float(np.median(errors)))
        if row['points']<60 or row['endpoint_inlier_fraction_within10cm']<.9 or row['median_endpoint_error_m']>.05:
            raise ValueError('current_initial_pose_not_supported_by_laser:'+json.dumps(row))
        rows.append(row)
    if len(rows)!=3:raise ValueError('three_current_laser_samples_required')
    return dict(rows=rows,expected_initial_pose=pose.tolist(),
        model='configured initial pose independently tested against current laser endpoints and map',
        ground_truth=False,robot_commands_sent=False)

with RobotLock():
    inputs=check_inputs(cfg)
    before=score(base/'apple-r11b-initial-scans-20261018-01.json')
    out.mkdir(exist_ok=False);write_json(out/'independent-initial-laser-check.json',before)
    subprocess.run([sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(out/'before-ready.json')],check=True)
    ready=json.loads((out/'before-ready.json').read_text())
    if not ready['ready'] or math.dist(ready['base_map_pose'][:2],pose[:2])>.02 or abs(ready['base_map_pose'][2]-pose[2])>.02:
        raise ValueError('reviewed_initial_empty_idle_state_changed')
    old_child=Path('/proc/399/cmdline').read_bytes().replace(b'\0',b' ').decode()
    old_parent=Path('/proc/398/cmdline').read_bytes().replace(b'\0',b' ').decode()
    if ('install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2/handyman_initial_pose_localizer' not in old_child
        or 'ros2 run handyman_rebuild_ros2 handyman_initial_pose_localizer' not in old_parent
        or os.getpgid(399)!=398 or os.getpgid(398)!=398):
        raise ValueError('exact_old_initial_localizer_identity_changed_no_stop_performed')
    intent=dict(status='replacing_inspected_idle_initial_anchor',old_launcher_pid=398,old_localizer_pid=399,
        old_command=old_child,expected_initial_pose=pose.tolist(),robot_commands_sent=False,
        old_numeric_anchor_reused=False,input_files_audit=inputs)
    write_json(out/'startup.json',intent)
    os.killpg(398,signal.SIGINT)
    deadline=time.monotonic()+5
    while (Path('/proc/398').exists() or Path('/proc/399').exists()) and time.monotonic()<deadline:time.sleep(.05)
    if Path('/proc/398').exists() or Path('/proc/399').exists():raise ValueError('old_localizer_not_exited_do_not_start_duplicate')
    command=['ros2','run','handyman_rebuild_ros2','handyman_initial_pose_localizer','--ros-args',
        '-p','map_frame:=map','-p','odom_frame:=odom','-p','base_frame:=base_footprint']
    for key in ('x','y','yaw'):command+=['-p','initial_pose.'+key+':='+str(float(initial[key]))]
    with (out/'new-localizer.log').open('x') as stream:
        child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    intent.update(new_localizer_launcher_pid=child.pid,new_command=command)
    write_json(out/'startup.json',intent)
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        if child.poll() is not None:raise ValueError('new_localizer_exited_do_not_move')
        if 'Aligned' in (out/'new-localizer.log').read_text(errors='replace'):break
        time.sleep(.1)
    else:raise ValueError('new_localizer_not_aligned_do_not_move')
    subprocess.run([sys.executable,str(base/'source-table-boundary-audit-20261017-01/capture_table_localization_audit.py'),
        '--output',str(out/'new-anchor-scans.json')],check=True)
    after=score(out/'new-anchor-scans.json');write_json(out/'new-anchor-laser-check.json',after)
    subprocess.run([sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(out/'ready.json')],check=True)
    intent.update(status='new_scene_initial_localization_and_idle_readiness_verified',
        independent_laser_before=before,independent_laser_after=after,robot_commands_sent=False)
    write_json(out/'completion.json',intent);print(json.dumps(intent),flush=True)
