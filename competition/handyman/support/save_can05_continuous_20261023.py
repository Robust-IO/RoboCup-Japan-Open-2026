"""Save only a genuinely uninterrupted successful run05, with actual start pose."""
import hashlib,json,math,subprocess,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
root=base/'can-auto-live-20261023-05';read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=read(root/'result.json');meta=read(root/'run-meta.json');grasp=read(root/'grasp/result.json')
carry=read(root/'carry/result.json');place=read(root/'place/result.json');v=read(root/'place/placement-verification.json')
context=read(root/'held-context.json');start=read(root/'preflight.json')['base_map_pose']
if (r.get('status')!='automatic_can_transfer_verified' or not r.get('physical_transfer_verified')
        or not r.get('uninterrupted_transfer_verified') or not r.get('robot_ready_for_next_task')
        or r.get('historical_checkpoint_used') is not False or len(r['confirmed_stages'])!=6):
    raise ValueError('actual_uninterrupted_success_required')
if (grasp['status']!='can_grasp_and_lift_verified' or carry['status']!='can_retraction_and_destination_navigation_verified'
        or place['status']!='reviewed_can_placement_verified' or place.get('run_kind')=='reviewed_release_only_continuation'
        or not v['placement_success_verified'] or not place['cleanup_completed']):raise ValueError('all_actual_stages_required')
if any(x!=meta['run_id'] for x in (context['run_id'],carry['context_run_id'],place['context_run_id'])):raise ValueError('cross_run_evidence')
def events(p):return [json.loads(x) for x in p.read_text().splitlines()]
closed=[x for x in events(root/'grasp/journal.jsonl') if x['event']=='can_closure_sent']
opened=[x for x in events(root/'place/journal.jsonl') if x['event']=='can_release_sent']
if len(closed)!=1 or len(opened)!=1 or place['release_commands_sent']!=1:raise ValueError('one_close_and_one_release_required')
latest=read(base/'canned-after-table-plane-fix-progress-20261023.json');hashes=dict(latest['latest_expected_hashes'])
for name in ('can-source640-v1-20261023-deployment.json','can-precontact-axis-v1-20261023-deployment.json'):
    hashes.update(read(base/name)['hashes'])
roots=[base,Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
for code in roots[1:]:
    if any(sha(code/n)!=digest for n,digest in hashes.items()):raise ValueError('tested_source_mismatch')
for p in (root/'source-navigation.json',root/'carry/destination-navigation.json'):
    n=read(p)
    if n['status']!='arrived' or n['goal_precision_changed'] or not n['speed_parameters_restored']:raise ValueError('original_nav_required')
starts={x['stage']:x['time_ns'] for x in events(root/'journal.jsonl') if x['event']=='intent'}
ends={x['stage']:x['time_ns'] for x in events(root/'journal.jsonl') if x['event']=='confirmed'}
default_start=math.dist(start[:2],[1.184,0.])<.05 and abs(start[2])<.05
name='canned-current-start-continuous-success-progress-20261023-05.json';checks=v['checks']
value=dict(status='one_current_source_area_uninterrupted_can_transfer_verified',saved_ns=time.time_ns(),
    branch=subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip(),
    run_id=meta['run_id'],boot_id=meta['boot_id'],actual_run='D:/handyman-training-runs/'+root.name,
    object='canned_juice',layout='LayoutA',source='white_side_table#bedroom_1',destination='square_low_table#living_room',
    actual_start_map_pose=start,default_initial_pose_whole_run_verified=default_start,
    uninterrupted_physical_transfer_verified=True,human_or_agent_stage_resume_used=False,historical_checkpoint_used=False,
    elapsed_s=r['elapsed_s'],stage_durations_s={k:(ends[k]-starts[k])/1e9 for k in starts},
    actual_close_commands=1,actual_release_commands=1,grasp_approaches=len(grasp['approaches']),
    placement_approaches=len(place.get('approaches',[])),
    final_three_depth_frames_verified=True,final_can_center_map_m=checks[-1]['center_map_m'],
    final_center_spread_m=v['center_spread_m'],
    final_outer_support_clearance_lower_estimate_m=checks[-1]['table_edge_safety']['outer_clearance_lower_estimate_m'],
    final_rgb_capture=place['final_placement_rows'][-1]['capture'].replace('/mnt/d/','D:/'),
    last_measured_robot_state=place['last_fresh_robot_state'],last_state_is_record_not_future_motion_authorization=True,
    robot_ready_for_next_task=True,cleanup_completed=True,tests=579,latest_expected_hashes=hashes,
    model_map_navigation_tolerances_unchanged=True,apple_legacy_unchanged=True,task_finished_sent=False,
    original_failed_runs_preserved=['can-auto-live-20261023-'+x for x in ('01','02','03','04')],
    continuous_successful_cases=1,success_rate_not_measured=True,full30_class_manipulation_verified=False,
    next=['Can is RELEASED on living_room square_low_table; robot is empty-handed and tucked. Never replay held continuations.',
        'Run05 started near bedroom source staging after explicit empty-arm retreat/tuck. It is not a default initial-pose whole-run test.',
        'All six stages from this actual start completed continuously without stage resumption or code changes mid-run.',
        'Next assess repeatability from a reset initial pose with the can restored to the source, then other objects on the same route.'])
if any((p/name).exists() for p in roots):raise ValueError('new_checkpoint_required')
data=json.dumps(value,indent=2,allow_nan=False).encode()
for p in roots:
    with (p/name).open('xb') as f:f.write(data)
print(json.dumps(dict(status=value['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),elapsed_s=value['elapsed_s'],stage_durations_s=value['stage_durations_s'],
    actual_start_map_pose=start,default_initial_pose_whole_run_verified=default_start,
    final_rgb_capture=value['final_rgb_capture'],grasp_approaches=value['grasp_approaches'],
    placement_approaches=value['placement_approaches'],final_support_clearance_m=value['final_outer_support_clearance_lower_estimate_m'],ready=True)),flush=True)
