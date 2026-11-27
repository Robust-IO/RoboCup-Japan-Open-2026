"""Record a successful SINGLE continuous run, only after actual placement."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
roots=[base,Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
root=base/'can-auto-live-20261023-01';read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=read(root/'result.json');meta=read(root/'run-meta.json');grasp=read(root/'grasp/result.json')
carry=read(root/'carry/result.json');place=read(root/'place/result.json');verification=read(root/'place/placement-verification.json')
if (r.get('status')!='automatic_can_transfer_verified' or not r.get('physical_transfer_verified')
        or not r.get('uninterrupted_transfer_verified') or not r.get('robot_ready_for_next_task')):
    raise ValueError('complete_continuous_physical_can_trial_required')
if (len(r['confirmed_stages'])!=6 or grasp['status']!='can_grasp_and_lift_verified'
        or carry['status']!='can_retraction_and_destination_navigation_verified'
        or place['status']!='reviewed_can_placement_verified' or not verification['placement_success_verified']
        or place['release_commands_sent']!=1 or not place['cleanup_completed']):
    raise ValueError('actual_stage_evidence_incomplete')
events=[json.loads(line) for line in (root/'journal.jsonl').read_text().splitlines()]
grasp_events=[json.loads(line) for line in (root/'grasp/journal.jsonl').read_text().splitlines()]
closures=[e for e in grasp_events if e['event']=='can_closure_sent']
if len(closures)!=1:raise ValueError('exactly_one_actual_closure_required')
context=read(root/'held-context.json')
if context['run_id']!=meta['run_id'] or carry['context_run_id']!=meta['run_id'] or place['context_run_id']!=meta['run_id']:
    raise ValueError('stages_not_bound_to_same_current_run')
source_nav=read(root/'source-navigation.json');dest_nav=read(root/'carry/destination-navigation.json')
for nav in (source_nav,dest_nav):
    if nav['status']!='arrived' or nav['goal_precision_changed'] or not nav['speed_parameters_restored']:
        raise ValueError('original_navigation_result_not_preserved')
deployment=read(base/'can-continuous-entry-v1-20261023-deployment.json')
for path in roots[1:]:
    if any(sha(path/name)!=digest for name,digest in deployment['hashes'].items()):raise ValueError('entry_code_changed_mid_trial')
starts={e['stage']:e['time_ns'] for e in events if e['event']=='intent'}
ends={e['stage']:e['time_ns'] for e in events if e['event']=='confirmed'}
name='canned-continuous-success-progress-20261023.json'
if any((path/name).exists() for path in roots):raise ValueError('new_checkpoint_required')
checks=verification['checks']
value=dict(status='one_fresh_start_continuous_can_transfer_verified',saved_ns=time.time_ns(),
    branch=subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip(),boot_id=meta['boot_id'],
    run_id=meta['run_id'],actual_run='D:/handyman-training-runs/'+root.name,
    object='canned_juice',layout='LayoutA',source='white_side_table#bedroom_1',destination='square_low_table#living_room',
    elapsed_s=r['elapsed_s'],stage_durations_s={k:(ends[k]-starts[k])/1e9 for k in starts},
    one_parent_entry_invoked=True,uninterrupted_physical_transfer_verified=True,historical_checkpoint_used=False,
    original_route_and_navigation_tolerances_preserved=True,navigation_time_limit_added=False,
    closure_commands=1,grasp_approaches=len(grasp['approaches']),placement_approaches=len(place.get('approaches',[])),release_commands=1,
    can_grasp_lift_verified=True,held_retraction_verified=True,destination_navigation_verified=True,placement_verified=True,
    final_can_center_map_m=checks[-1]['center_map_m'],final_center_spread_m=verification['center_spread_m'],
    final_outer_support_clearance_lower_estimate_m=checks[-1]['table_edge_safety']['outer_clearance_lower_estimate_m'],
    final_rgb_capture=place['final_placement_rows'][-1]['capture'].replace('/mnt/d/','D:/'),
    robot_ready_for_next_task=True,last_measured_robot_state=place['last_fresh_robot_state'],
    last_state_is_record_not_future_motion_authorization=True,task_finished_sent=False,
    full30_class_manipulation_verified=False,success_rate_not_measured=True,continuous_successful_cases=1,
    next=['Can is RELEASED on living_room square_low_table; robot is empty-handed and tucked. Never rerun a held continuation.',
        'Retain this whole run and prior staged results. Repeat a fresh-start same-route trial to assess repeatability before changing object or route.',
        'Competition task parsing/protocol success and general manipulation across30 classes remain separate work; do not send Task_finished solely from this development result.'])
data=json.dumps(value,indent=2,allow_nan=False).encode()
for path in roots:
    with (path/name).open('xb') as f:f.write(data)
print(json.dumps(dict(status=value['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),elapsed_s=value['elapsed_s'],stage_durations_s=value['stage_durations_s'],
    grasp_approaches=value['grasp_approaches'],placement_approaches=value['placement_approaches'],
    final_center=value['final_can_center_map_m'],ready=True)),flush=True)
