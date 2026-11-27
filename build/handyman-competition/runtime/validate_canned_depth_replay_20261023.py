"""Re-evaluate actual RGBD with cached current-model detections; no ROS actions."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args();args.output.mkdir(exist_ok=False)
sys.path.insert(0,str(args.candidate));sys.path.insert(0,str(args.candidate/'scripts'))
import numpy as np
from PIL import Image
import yaml
from apple_rgbd_candidate import estimate as legacy_estimate
from object_surface_geometry import estimate
from inspect_manipulation_capture import inspect
from apple_transfer_auto_plan import map_point
from scripts.search_room_gate import inside

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
registry=base/'active_detection_model.json';cfg=json.loads(registry.read_text());registry_hash=sha(registry)
if sha(Path(cfg['weights']))!=cfg['sha256']:raise ValueError('model_hash_mismatch')
trial=base/'automatic-apple-r19-20261022-01'
capture=Path(json.loads((trial/'measured-grasp-plan.json').read_text())['capture'])
snapshot=json.loads((capture/'snapshot.json').read_text());observation=json.loads((capture/'observation.json').read_text())
apples=[d for d in observation['detections'] if d['name']=='apple' and d['confidence']>=.8]
if len(apples)!=1:raise ValueError('reviewed_apple_replay_required')
old_hashes={n:sha(capture/n) for n in ('rgb.png','depth_m.npy','snapshot.json','observation.json')}
rgb=np.asarray(Image.open(capture/'rgb.png').convert('RGB'));depth=np.load(capture/'depth_m.npy',allow_pickle=False)
old=legacy_estimate(rgb,depth,snapshot,apples[0],geometry_proposal_only=True)
wrapped=estimate(rgb,depth,snapshot,apples[0],'apple')
if old!=wrapped['legacy_candidate']:raise ValueError('apple_numeric_result_changed')
source=base/'canned-source-observation-20261023-01'
analyses=base/'canned-source-analysis-20261023-02'
env=yaml.safe_load(Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/src/handyman_rebuild_ros2/config/environments/layout_a.yaml').read_text())
prior=json.loads((analyses/'result.json').read_text())
if prior['model_sha256']!=cfg['sha256']:raise ValueError('cached_detection_model_mismatch')
rows=[];centers=[]
for f in sorted((source/'cameras').glob('*/snapshot.json')):
    cap=f.parent;snap=json.loads(f.read_text());saved=analyses/(cap.name+'-analysis.json')
    hashes={n:sha(cap/n) for n in ('rgb.png','depth_m.npy','snapshot.json')};saved_hash=sha(saved)
    stored=json.loads(saved.read_text());result=inspect(cap,stored['detections'],'canned_juice')
    if result['source_sha256']!=stored['source_sha256']:raise ValueError('detection_rgbd_provenance_mismatch')
    if not result['unique_geometry_candidate']:raise ValueError('actual_can_frame_fit_failed:'+str(cap)+':'+json.dumps(result['rejections']))
    item=result['geometry_candidates'][0];geom=item['geometry'];center=map_point(snap,np.array(geom['center_m']))
    room=[name for name,r in env['rooms'].items() if inside(*center[:2],r['region'])]
    if room!=['bedroom'] or not item['identity_score_passed']:raise ValueError('actual_can_class_or_room_mismatch')
    item.update(center_map_m=center.tolist(),room_membership=room)
    centers.append(center)
    (args.output/(cap.name+'-analysis.json')).write_text(json.dumps(result,indent=2))
    rows.append(dict(frame=cap.name,detection=item['detection'],geometry=geom,center_map_m=center.tolist(),room_membership=room))
    if any(sha(cap/n)!=digest for n,digest in hashes.items()) or sha(saved)!=saved_hash:raise ValueError('recording_modified')
if len(rows)!=3:raise ValueError('three_actual_frames_required')
spread=float(np.max(np.linalg.norm(np.array(centers)-np.mean(centers,axis=0),axis=1)))
if spread>.015:raise ValueError('observed_can_centers_not_repeatable')
if sha(registry)!=registry_hash or sha(Path(cfg['weights']))!=cfg['sha256']:raise ValueError('model_changed')
if any(sha(capture/n)!=v for n,v in old_hashes.items()):raise ValueError('apple_recording_changed')
report=dict(status='actual_can_rgbd_replay_and_apple_numeric_replay_passed',
    apple_legacy_numeric_result_identical=True,can_three_actual_rgbd_frames_validated=True,
    source_run=str(source),cached_detections=str(analyses),model_sha256=cfg['sha256'],
    rows=rows,center_spread_radius_m=spread,source_recordings_unchanged=True,
    previous_radius_rejections_retained=True,robot_commands_sent=False,
    not_a_fresh_motion_lease=True,physical_grasp_or_transfer_verified=False)
(args.output/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
