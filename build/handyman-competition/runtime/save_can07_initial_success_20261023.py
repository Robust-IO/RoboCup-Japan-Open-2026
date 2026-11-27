"""Save run07 only after continuous success from the recreated initial pose."""
import hashlib,json,math,subprocess,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
root=base/'can-auto-live-20261023-07';read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=read(root/'result.json');meta=read(root/'run-meta.json');start=read(root/'preflight.json')['base_map_pose']
grasp=read(root/'grasp/result.json');carry=read(root/'carry/result.json');place=read(root/'place/result.json')
v=read(root/'place/placement-verification.json');context=read(root/'held-context.json')
expected=['preflight','navigate_source','head_view','grasp_lift','carry_destination','place_verify']
if (r.get('status')!='automatic_can_transfer_verified' or r.get('mode')!='live'
        or not r.get('physical_transfer_verified') or not r.get('uninterrupted_transfer_verified')
        or not r.get('robot_ready_for_next_task') or r.get('confirmed_stages')!=expected
        or r.get('historical_checkpoint_used') is not False or r.get('automatic_resume') is not False):
    raise ValueError('complete_continuous_live_run_required')
if math.dist(start[:2],[1.184,0.])>.05 or abs(start[2])>.05:raise ValueError('default_initial_start_required')
if (grasp.get('status')!='can_grasp_and_lift_verified' or carry.get('status')!='can_retraction_and_destination_navigation_verified'
        or place.get('status')!='reviewed_can_placement_verified' or not v.get('placement_success_verified')
        or not place.get('cleanup_completed') or place.get('run_kind')=='reviewed_release_only_continuation'):
    raise ValueError('actual_current_stage_evidence_required')
if any(x!=meta['run_id'] for x in (context['run_id'],carry['context_run_id'],place['context_run_id'])):raise ValueError('cross_run_evidence')
def events(p):return [json.loads(x) for x in p.read_text().splitlines()]
g_events=events(root/'grasp/journal.jsonl');p_events=events(root/'place/journal.jsonl')
closes=[x for x in g_events if x['event']=='can_closure_sent'];releases=[x for x in p_events if x['event']=='can_release_sent']
if len(closes)!=1 or len(releases)!=1 or place.get('release_commands_sent')!=1:raise ValueError('one_grasp_and_one_release_required')
if len(grasp.get('lift_verification',[]))!=3 or len(place.get('final_placement_rows',[]))!=3:
    raise ValueError('three_current_lift_and_placement_frames_required')
previous=read(base/'canned-current-start-continuous-success-progress-20261023-05.json')
deployment=read(base/'can-coherent-startup-v1-20261023-deployment.json')
hashes=dict(previous['latest_expected_hashes'],**deployment['hashes'])
roots=[base,Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
for code in roots[1:]:
    if any(sha(code/n)!=digest for n,digest in hashes.items()):raise ValueError('source_changed_since_previous_test')
for path in (root/'source-navigation.json',root/'carry/destination-navigation.json'):
    nav=read(path)
    if nav.get('status')!='arrived' or nav.get('goal_precision_changed') or not nav.get('speed_parameters_restored'):
        raise ValueError('original_navigation_and_restoration_required')
journal=events(root/'journal.jsonl');starts={x['stage']:x['time_ns'] for x in journal if x['event']=='intent'}
ends={x['stage']:x['time_ns'] for x in journal if x['event']=='confirmed'}
checks=v['checks'];last=checks[-1]
value=dict(status='default_initial_pose_uninterrupted_can_transfer_verified',saved_ns=time.time_ns(),
    branch=subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip(),
    run_id=meta['run_id'],boot_id=meta['boot_id'],actual_run='D:/handyman-training-runs/'+root.name,
    object='canned_juice',layout='LayoutA',source='white_side_table#bedroom_1',destination='square_low_table#living_room',
    actual_start_map_pose=start,default_initial_pose_whole_run_verified=True,
    uninterrupted_physical_transfer_verified=True,human_or_agent_stage_resume_used=False,historical_checkpoint_used=False,
    elapsed_s=r['elapsed_s'],stage_durations_s={k:(ends[k]-starts[k])/1e9 for k in expected},
    actual_close_commands=1,actual_release_commands=1,grasp_approaches=len(grasp['approaches']),
    placement_approaches=len(place.get('approaches',[])),
    final_three_depth_frames_verified=True,final_can_center_map_m=last['center_map_m'],
    final_center_spread_m=v['center_spread_m'],
    final_outer_support_clearance_lower_estimate_m=last['table_edge_safety']['outer_clearance_lower_estimate_m'],
    final_rgb_capture=place['final_placement_rows'][-1]['capture'].replace('/mnt/d/','D:/'),
    last_measured_robot_state=place['last_fresh_robot_state'],last_state_is_record_not_future_motion_authorization=True,
    robot_ready_for_next_task=True,cleanup_completed=True,tests=deployment['tests'],latest_expected_hashes=hashes,
    previous_source_area_continuous_success=previous['actual_run'],
    model_map_navigation_tolerances_unchanged=True,apple_default_behavior_unchanged=True,task_finished_sent=False,
    initial_pose_recreated_by_empty_return=True,unity_restarted_immediately_before_run=False,
    initial_pose_preparation='D:/handyman-training-runs/can06-return-initial-20261023-01',
    prior_failed_initial_start_run_preserved='D:/handyman-training-runs/can-auto-live-20261023-06',
    startup_fix='Can-only opt-in latches an actual still-fresh joint-time transform during the original150ms stability check; latest joint feedback and movement checks remain unchanged.',
    initial_start_successful_cases_on_this_code=1,success_rate_not_measured=True,
    full30_class_manipulation_verified=False,
    next=['Can is RELEASED on living_room square_low_table; robot is empty-handed and tucked. Do not replay held continuations.',
        'Default initial position to source navigation, actual new RGBD identity, one grasp/lift, original return navigation, one release and final placement verification all completed in one process.',
        'Run06 stopped before any arm command at a joint/TF startup race. Its failed verdict remains preserved. The empty robot was returned along the original route and aligned to the initial pose before this new run07.',
        'All six run07 stages completed continuously without stage resumption or mid-run code changes. This establishes one fixed-route can case, not all30 objects or a success-rate target.',
        'Next repeat for reliability or develop the next object adapter on the same route, preserving successful apple/can entries. Moderator Task_finished integration remains separate.'])
name='canned-initial-start-continuous-success-progress-20261023-07.json'
if any((p/name).exists() for p in roots):raise ValueError('new_checkpoint_required')
data=json.dumps(value,indent=2,allow_nan=False).encode()
for p in roots:
    with (p/name).open('xb') as f:f.write(data)
print(json.dumps(dict(status=value['status'],checkpoint='D:/handyman-training-runs/'+name,
    sha256=hashlib.sha256(data).hexdigest(),elapsed_s=value['elapsed_s'],stage_durations_s=value['stage_durations_s'],
    actual_start_map_pose=start,final_rgb_capture=value['final_rgb_capture'],
    grasp_approaches=value['grasp_approaches'],placement_approaches=value['placement_approaches'],
    final_support_clearance_m=value['final_outer_support_clearance_lower_estimate_m'],ready=True)),flush=True)
