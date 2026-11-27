"""Replay historical REAL captures through the new handoff. Never imports ROS.

Canonical fixture copies are explicitly marked recorded_replay and cannot be
loaded as a live context. Original failed verdicts/raw recordings are untouched.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys

p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
sys.path.insert(0,str(args.candidate))
import cv2
import numpy as np
from apple_transfer_automation import write_json
from can_transfer_context import build_context,load_context
from canned_grasp_geometry import measure_near
from held_can_appearance import build_appearance,assess
from canned_place_geometry import verify_placed,make_plan

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=args.output;root.mkdir(parents=True,exist_ok=False)
read=lambda path:json.loads(path.read_text());sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
historic=read(base/'canned-grasp-lift-live-progress-20261023.json');replay=historic['recorded_replay'];original=Path(replay['source_trial'])
old_result=read(original/'result.json');source_before=Path(replay['before_capture'])
watched={str(original/'result.json'):sha(original/'result.json')}
case=root/'recorded-run';grasp=case/'grasp';grasp.mkdir(parents=True)
before=grasp/'captures/preclose';before.mkdir(parents=True)
def copy_raw(source,destination):
    for name in ('rgb.png','depth_m.npy','snapshot.json'):
        path=source/name;watched[str(path)]=sha(path);shutil.copyfile(path,destination/name)
        if sha(destination/name)!=watched[str(path)]:raise ValueError('raw_copy_mismatch')
copy_raw(source_before,before);write_json(before/'can-measurement.json',replay['before_measurement'])
rows=[]
for index in (0,6,12):
    row=replay['rows'][index];source=Path(row['capture']);dest=grasp/f'captures/after-lift-{index:02d}';dest.mkdir()
    copy_raw(source,dest);snapshot=read(dest/'snapshot.json');old_m=row['measurement']
    actual=measure_near(np.load(dest/'depth_m.npy',allow_pickle=False),snapshot,old_m['center_m'],
        historic['can_footprint_radius_m'],historic['can_height_m'])
    write_json(dest/'can-measurement.json',actual);rows.append(dict(row,capture=str(dest)))
meta=dict(run_id='recorded-real-can-handoff',mode='recorded_replay',object='canned_juice',
    boot_id=historic['boot_id'],started_ns=replay['before_measurement']['stamp_ns']-1)
write_json(case/'run-meta.json',meta)
fixture=dict(status='can_grasp_and_lift_verified',grasp_success_verified=True,task_id='grasp',
    evidence_mode='recorded_replay_only',original_failed_verdict_preserved=str(original/'result.json'),
    close=dict(old_result['close'],before_capture=str(before)),lift=old_result['lift'],
    held_opening_rad=historic['held_opening_rad'],lift_verification=rows,reference=dict(height_m=historic['can_height_m']),
    grasp_plan=dict(contact_start_lift_m=historic['last_measured_robot_state']['joints'][0]-.1))
write_json(grasp/'result.json',fixture);context=build_context(case);reference=read(context)
try:load_context(context,case/'carry')
except ValueError:pass
else:raise ValueError('historical_replay_context_accepted_for_live_motion')
offset_error=float(np.linalg.norm(np.asarray(reference['held_center_minus_palm_base_m'])-historic['held_center_minus_palm_base_m']))
if offset_error>.005:raise ValueError('new_handoff_reference_disagrees_with_measured_actual_grasp')
source=Path(reference['source_capture'])
appearance=build_appearance(cv2.cvtColor(cv2.imread(str(source/'rgb.png')),cv2.COLOR_BGR2RGB),
    np.load(source/'depth_m.npy',allow_pickle=False),read(source/'snapshot.json'),read(source/'can-measurement.json'))
carry=base/'canned-carry-live-20261023-01'
tucked=sorted((carry/'captures').glob('after-retract-rejected-*'))
tucked+=sorted((base/'tucked-can-rgb-probe-20261023-01/captures').glob('frame-*'))
for path in tucked:
    evidence=assess(cv2.cvtColor(cv2.imread(str(path/'rgb.png')),cv2.COLOR_BGR2RGB),np.load(path/'depth_m.npy',allow_pickle=False),
        read(path/'snapshot.json'),reference,appearance)
    if not evidence['current_rgb_appearance_verified'] or evidence['measured_3d_object_position_available']:
        raise ValueError('incorrect_new_reference_near_clip_semantics')
measured=read(base/'can-destination-measured-20261023-01/result.json');cfg=read(args.candidate/'config/apple_transfer_auto_v1.json')
plans=[]
for row in measured['rows']:
    path=Path(row['capture']);plan=make_plan(np.load(path/'depth_m.npy',allow_pickle=False),read(path/'snapshot.json'),
        measured['table'],row['plane'],reference,cfg,'recorded-new-context-plan')
    gap=plan['palm_height_m']+reference['held_center_minus_palm_base_m'][2]-plan['height_m']/2-plan['table_height_at_release_m']
    if not plan['table_edge_safety']['safe'] or abs(gap-.02)>1e-8:raise ValueError('new_context_wrong_actual_table_can_bottom')
    plans.append(dict(xy=plan['release_xy_map_m'],gap_m=gap))
place=base/'canned-place-live-20261023-01';outcome=read(place/'result.json');placed_rows=[]
for row in outcome['final_placement_rows']:
    placed_rows.append(read(Path(row['capture'])/'observation.json'))
verified=verify_placed(placed_rows,read(place/'placement-plan.json'),outcome['last_fresh_robot_state']['joints'],cleanup_completed=True)
if not verified['placement_success_verified']:raise ValueError('actual_placement_replay_not_verified')
if any(sha(Path(path))!=digest for path,digest in watched.items()):raise ValueError('original_recording_changed')
result=dict(status='actual_recorded_handoff_carry_and_placement_passed',
    recorded_grasp_lift_frames=3,new_reference_offset_error_m=offset_error,
    recorded_tucked_rgb_frames=len(tucked),recorded_table_frames=len(plans),recorded_final_placement_frames=len(placed_rows),
    replay_context_rejected_for_live=True,original_failed_verdict_preserved=True,
    robot_commands_sent=False,new_continuous_physical_transfer_verified=False,plans=plans)
write_json(root/'result.json',result);print(json.dumps(result),flush=True)
