"""Save success only after a complete new run with the corrected code."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
root=base/'can-auto-live-20261023-02';read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=read(root/'result.json');meta=read(root/'run-meta.json');grasp=read(root/'grasp/result.json');carry=read(root/'carry/result.json')
place=read(root/'place/result.json');verification=read(root/'place/placement-verification.json');context=read(root/'held-context.json')
if (r.get('status')!='automatic_can_transfer_verified' or not r.get('physical_transfer_verified')
        or not r.get('uninterrupted_transfer_verified') or not r.get('robot_ready_for_next_task')
        or r.get('historical_checkpoint_used') is not False):raise ValueError('complete_new_uninterrupted_live_trial_required')
if (len(r['confirmed_stages'])!=6 or grasp['status']!='can_grasp_and_lift_verified'
        or carry['status']!='can_retraction_and_destination_navigation_verified'
        or place['status']!='reviewed_can_placement_verified' or place.get('run_kind')=='reviewed_release_only_continuation'
        or not verification['placement_success_verified'] or not place['cleanup_completed']):raise ValueError('actual_continuous_stage_evidence_required')
if any(v!=meta['run_id'] for v in (context['run_id'],carry['context_run_id'],place['context_run_id'])):
    raise ValueError('cross_run_evidence_not_allowed')
def events(path):return [json.loads(line) for line in path.read_text().splitlines()]
closed=[e for e in events(root/'grasp/journal.jsonl') if e['event']=='can_closure_sent']
opened=[e for e in events(root/'place/journal.jsonl') if e['event']=='can_release_sent']
if len(closed)!=1 or len(opened)!=1 or place['release_commands_sent']!=1:raise ValueError('one_close_and_one_open_required')
latest=read(base/'canned-after-two-fixes-progress-20261023.json');hashes=latest['latest_expected_hashes']
roots=[base,Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
for code in roots[1:]:
    if any(sha(code/name)!=digest for name,digest in hashes.items()):raise ValueError('current_source_differs_from_tested_entry')
for p in (root/'source-navigation.json',root/'carry/destination-navigation.json'):
    nav=read(p)
    if nav['status']!='arrived' or nav['goal_precision_changed'] or not nav['speed_parameters_restored']:
        raise ValueError('original_navigation_success_required')
journal=events(root/'journal.jsonl');starts={e['stage']:e['time_ns'] for e in journal if e['event']=='intent'}
ends={e['stage']:e['time_ns'] for e in journal if e['event']=='confirmed'};checks=verification['checks']
value=dict(status='one_fresh_start_uninterrupted_can_transfer_verified',saved_ns=time.time_ns(),run_id=meta['run_id'],boot_id=meta['boot_id'],
    branch=subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip(),
    actual_run='D:/handyman-training-runs/'+root.name,object='canned_juice',layout='LayoutA',
    source='white_side_table#bedroom_1',destination='square_low_table#living_room',elapsed_s=r['elapsed_s'],
    stage_durations_s={k:(ends[k]-starts[k])/1e9 for k in starts},
    uninterrupted_physical_transfer_verified=True,human_or_agent_stage_resume_used=False,historical_checkpoint_used=False,
    actual_closure_commands=1,actual_release_commands=1,grasp_approaches=len(grasp['approaches']),
    placement_approaches=len(place.get('approaches',[])),
    initial_grasped_can_axis_base_m=context.get('can_axis_base_m'),
    final_can_center_map_m=checks[-1]['center_map_m'],final_center_spread_m=verification['center_spread_m'],
    final_outer_support_clearance_lower_estimate_m=checks[-1]['table_edge_safety']['outer_clearance_lower_estimate_m'],
    final_rgb_capture=place['final_placement_rows'][-1]['capture'].replace('/mnt/d/','D:/'),
    cleanup_completed=True,robot_ready_for_next_task=True,last_measured_robot_state=place['last_fresh_robot_state'],
    last_state_is_record_not_future_motion_authorization=True,tests=latest['tests'],latest_expected_hashes=hashes,
    model_map_navigation_route_tolerances_unchanged=True,navigation_time_limit_added=False,task_finished_sent=False,
    continuous_successful_cases=1,success_rate_not_measured=True,full30_class_manipulation_verified=False,
    prior_failed_full_run_preserved='D:/handyman-training-runs/can-auto-live-20261023-01',
    next=['Can is now RELEASED on living_room square_low_table. Robot is empty-handed and tucked; never replay held continuations.',
        'This corrected fixed-route can entry completed once from a fresh initial pose without stage resumption or code changes mid-run.',
        'Repeat to assess stability before changing objects or routes. This one case does not establish a success-rate target,30-class manipulation, or the Moderator protocol.'])
name='canned-continuous-success-progress-20261023-02.json'
if any((p/name).exists() for p in roots):raise ValueError('new_checkpoint_required')
data=json.dumps(value,indent=2,allow_nan=False).encode()
for p in roots:
    with (p/name).open('xb') as f:f.write(data)
print(json.dumps(dict(status=value['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),elapsed_s=value['elapsed_s'],stage_durations_s=value['stage_durations_s'],
    final_rgb_capture=value['final_rgb_capture'],grasp_approaches=value['grasp_approaches'],placement_approaches=value['placement_approaches'],ready=True)),flush=True)
