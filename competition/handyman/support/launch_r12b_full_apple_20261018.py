"""Launch ONE new full live trial from verified same-scene idle state; no replay."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import uuid

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
import sys
sys.path.insert(0,str(code))
from hand_camera_ray import rotation

out=base/'automatic-apple-r12b-20261018-01'
console=base/'automatic-apple-r12b-20261018-01.console.log'
startup=base/'automatic-apple-r12b-20261018-01.startup.json'
if any(p.exists() for p in (out,console,startup)):raise ValueError('new_trial_output_required')
precheck=base/'apple-r12b-live-precheck-20261018-01'
pre=json.loads((precheck/'result.json').read_text())
if any(pre['exit_codes'].values()) or pre['readiness'].get('ready') is not True:raise ValueError('idle_precheck_not_passed')
snapshot=json.loads((Path(pre['last_capture'])/'snapshot.json').read_text())
previous=json.loads((base/'apple-r11b-reviewed1280-transfer-20261018-01/placed-final/000005/snapshot.json').read_text())
for name in ('odom_from_base_at_depth','map_from_base_at_depth'):
    current=snapshot['tracking_transforms'][name];old=previous['tracking_transforms'][name]
    rc=rotation(current['quaternion']);ro=rotation(old['quaternion'])
    angle=math.atan2(math.sin(math.atan2(rc[1,0],rc[0,0])-math.atan2(ro[1,0],ro[0,0])),
                     math.cos(math.atan2(rc[1,0],rc[0,0])-math.atan2(ro[1,0],ro[0,0])))
    if math.dist(current['translation'][:2],old['translation'][:2])>.02 or abs(angle)>.02:
        raise ValueError('scene_or_robot_changed_requires_new_localization_review:'+name)

cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text());p=cfg['policy']
if (p.get('near_grasp_geometry_handoff') is not True or p['near_grasp_geometry_gap_m']!=.15
        or p['placement_approach_mode']!='single' or p['max_placement_step_m'] is not None
        or p['edge_margin_m']!=.03 or p['edge_uncertainty_m']!=.01):raise ValueError('requested_policy_not_installed')
deployment=json.loads((base/'apple-near-geometry-r12b-20261018-deployment.json').read_text())
if deployment['status']!='deployed_tests_and_recorded_replay_passed':raise ValueError('unverified_deployment')
for name,digest in deployment['hashes'].items():
    if hashlib.sha256((code/name).read_bytes()).hexdigest()!=digest:raise ValueError('code_changed:'+name)

map_path=Path('/__HANDYMAN_PACKAGE__/project/src/handyman_rebuild_ros2/maps/LayoutA/map.yaml')
meta=yaml.safe_load(map_path.read_text());grid=np.asarray(Image.open(map_path.parent/meta['image']))
if meta['origin'][2]!=0 or meta.get('negate',0)!=0:raise ValueError('unsupported_map')
dist=distance_transform_edt(~((255-grid.astype(float))/255>meta['occupied_thresh']))*meta['resolution']
correction=base/'apple-r12b-current-localization-20261018-01'
alignment=json.loads((correction/'completion.json').read_text())
if alignment.get('status')!='current_idle_localization_correction_published_and_readiness_verified':
    raise ValueError('reviewed_current_localization_correction_required')
current_publisher=Path('/proc')/str(alignment['new_anchor_pid'])/'cmdline'
if 'publish_r12b_reviewed_map_anchor_20261018.py' not in current_publisher.read_bytes().replace(b'\0',b' ').decode():
    raise ValueError('current_reviewed_map_anchor_not_running')
rows=[]
for scan in json.loads((correction/'new-scans.json').read_text())['scans']:
    ranges=np.array([np.nan if v is None else v for v in scan['ranges']]);angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    idx=np.flatnonzero(np.isfinite(ranges)&(ranges>max(.15,scan['range_min']))&(ranges<min(9.,scan['range_max'])))[::3]
    points=np.c_[ranges[idx]*np.cos(angles[idx]),ranges[idx]*np.sin(angles[idx]),np.zeros(len(idx))]
    tf=scan['base_from_scan'];points=points@rotation(tf['quaternion']).T+tf['translation']
    tf=scan['map_from_base'];points=points@rotation(tf['quaternion']).T+tf['translation']
    cells=np.floor((points[:,:2]-meta['origin'][:2])/meta['resolution']).astype(int)
    u=cells[:,0];v=grid.shape[0]-1-cells[:,1];valid=(u>=0)&(u<grid.shape[1])&(v>=0)&(v<grid.shape[0])
    errors=np.full(len(u),10.);errors[valid]=dist[v[valid],u[valid]]
    row=dict(stamp_ns=scan['stamp_ns'],points=len(errors),within_10cm_fraction=float(np.mean(errors<=.1)),median_error_m=float(np.median(errors)))
    if row['points']<60 or row['within_10cm_fraction']<.9 or row['median_error_m']>.05:
        raise ValueError('current_laser_map_alignment_not_supported:'+json.dumps(row))
    rows.append(row)
if len(rows)!=3:raise ValueError('three_current_scans_required')

task='apple-r12b-auto-'+uuid.uuid4().hex
command=['bash',str(code/'run_apple_transfer_auto.sh'),'--run','--confirm-layout','LayoutA',
    '--confirm-localization','--simulation-ignore-collision','--task-id',task,'--output',str(out)]
with console.open('x') as stream:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,
        env=dict(os.environ,ROS_LOG_DIR=str(base/'automatic-apple-r12b-20261018-01-roslogs')))
report=dict(pid=child.pid,task_id=task,command=command,output=str(out),console=str(console),
    precheck=str(precheck),laser_map_check=rows,localization='current same-scene laser-map correction applied without moving robot; fresh3-scan check passed',
    localization_completion=str(correction/'completion.json'),new_anchor_published=True,old_numeric_anchor_restored=False,full_new_trial=True,
    near_geometry_gap_m=.15,default_source_detector_imgsz=960,placement_mode='single',
    automatic_retry=False,old_journal_replayed=False,success_not_yet_known=True)
with startup.open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report),flush=True)
