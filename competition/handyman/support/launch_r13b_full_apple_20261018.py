"""One NEW full trial AFTER real no-motion source validation; never resume/replay."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
from hand_camera_ray import rotation
out=base/'automatic-apple-r13b-20261018-01'
precheck=base/'apple-r13b-live-precheck-20261018-01'
console=base/'automatic-apple-r13b-20261018-01.console.log'
startup=base/'automatic-apple-r13b-20261018-01.startup.json'
if any(p.exists() for p in (out,precheck,console,startup)):raise ValueError('new_trial_outputs_required')
if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('domain71_required')
source=base/'apple-capture-budget-r13b-source-readonly-20261018-01'
proof=json.loads((source/'summary.json').read_text());observed=json.loads((source/'result.json').read_text())
if (proof['status']!='live_source_pairing_identity_and_plans_verified'
    or observed['status']!='source_timing_readonly_passed' or observed['samples']<5
    or observed['all_grasp_plans_eligible'] is not True):raise ValueError('real_source_validation_required')
cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text());p=cfg['policy']
if (p.get('near_grasp_geometry_handoff') is not True or p['near_grasp_geometry_gap_m']!=.15
    or p['placement_approach_mode']!='single' or p['max_placement_step_m'] is not None
    or p['edge_margin_m']!=.03 or p['edge_uncertainty_m']!=.01):raise ValueError('requested_policy_required')
hashes={}
for name in ('apple-near-geometry-r12b-20261018-deployment.json','apple-capture-budget-r13-20261018-deployment.json',
             'apple-capture-budget-r13b-20261018-deployment.json'):
    deployed=json.loads((base/name).read_text())
    if not deployed['status'].startswith('deployed_tests'):raise ValueError('tested_deployment_required:'+name)
    hashes.update(deployed['hashes'])
for name,digest in hashes.items():
    if hashlib.sha256((code/name).read_bytes()).hexdigest()!=digest:raise ValueError('code_changed:'+name)
alignment=json.loads((base/'apple-r12b-current-localization-20261018-01/completion.json').read_text())
if alignment['status']!='current_idle_localization_correction_published_and_readiness_verified':raise ValueError('current_alignment_required')
anchor=Path('/proc')/str(alignment['new_anchor_pid'])/'cmdline'
if 'publish_r12b_reviewed_map_anchor_20261018.py' not in anchor.read_bytes().replace(b'\0',b' ').decode():raise ValueError('current_anchor_not_running')
with RobotLock():
    precheck.mkdir(exist_ok=False)
    for name,command in (
        ('scans',[sys.executable,str(base/'source-table-boundary-audit-20261017-01/capture_table_localization_audit.py'),'--output',str(precheck/'scans.json')]),
        ('ready',[sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(precheck/'ready.json')])):
        with (precheck/(name+'.log')).open('x') as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=25)
    ready=json.loads((precheck/'ready.json').read_text())
    if ready.get('ready') is not True:raise ValueError('current_empty_idle_readiness_required')
    if math.dist(ready['base_map_pose'][:2],observed['pose_after'][:2])>.02:raise ValueError('robot_or_map_changed_after_source_validation')
    meta_path=Path('/__HANDYMAN_PACKAGE__/project/src/handyman_rebuild_ros2/maps/LayoutA/map.yaml')
    meta=yaml.safe_load(meta_path.read_text());grid=np.asarray(Image.open(meta_path.parent/meta['image']))
    if meta['origin'][2]!=0 or meta.get('negate',0)!=0:raise ValueError('unsupported_map')
    distance=distance_transform_edt(~((255-grid.astype(float))/255>meta['occupied_thresh']))*meta['resolution']
    checks=[]
    for scan in json.loads((precheck/'scans.json').read_text())['scans']:
        ranges=np.array([np.nan if v is None else v for v in scan['ranges']]);angles=scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
        indices=np.flatnonzero(np.isfinite(ranges)&(ranges>max(.15,scan['range_min']))&(ranges<min(9.,scan['range_max'])))[::3]
        points=np.c_[ranges[indices]*np.cos(angles[indices]),ranges[indices]*np.sin(angles[indices]),np.zeros(len(indices))]
        for key in ('base_from_scan','map_from_base'):
            tf=scan[key];points=points@rotation(tf['quaternion']).T+tf['translation']
        cells=np.floor((points[:,:2]-meta['origin'][:2])/meta['resolution']).astype(int)
        u=cells[:,0];v=grid.shape[0]-1-cells[:,1];valid=(u>=0)&(u<grid.shape[1])&(v>=0)&(v<grid.shape[0])
        errors=np.full(len(u),10.);errors[valid]=distance[v[valid],u[valid]]
        check=dict(stamp_ns=scan['stamp_ns'],points=len(errors),within_10cm_fraction=float(np.mean(errors<=.1)),median_error_m=float(np.median(errors)))
        if check['points']<60 or check['within_10cm_fraction']<.9 or check['median_error_m']>.05:raise ValueError('current_laser_map_mismatch:'+json.dumps(check))
        checks.append(check)
    if len(checks)!=3:raise ValueError('three_current_scans_required')
task='apple-r13b-auto-'+uuid.uuid4().hex
command=['bash',str(code/'run_apple_transfer_auto.sh'),'--run','--confirm-layout','LayoutA','--confirm-localization',
    '--simulation-ignore-collision','--task-id',task,'--output',str(out)]
with console.open('x') as log:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,
        env=dict(os.environ,ROS_LOG_DIR=str(base/'automatic-apple-r13b-20261018-01-roslogs')))
report=dict(pid=child.pid,task_id=task,command=command,output=str(out),console=str(console),
    source_readonly_evidence=str(source/'summary.json'),precheck=str(precheck),laser_map_checks=checks,
    current_scene_anchor_rechecked=True,new_anchor_published=False,old_numeric_anchor_restored=False,
    full_new_trial=True,old_journal_replayed=False,automatic_retry=False,success_not_yet_known=True)
with startup.open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report),flush=True)
