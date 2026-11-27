"""Record actual staged transfer and final empty state; metadata update only."""
import hashlib,json,sys,tarfile,time
from pathlib import Path

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');roots=[code,repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,check_inputs
from canned_grasp_geometry import closure_error
from task_outcome_policy import robot_ready

run_names=['pepper-native-pregrasp-continuation-20261024-07','pepper-native-destination-place-20261024-10',
    'pepper-native-released-cleanup-20261024-11','pepper-native-placement-proof-20261024-12']
r7,r10,r11,r12=[read(base/n/'result.json') for n in run_names]
navigation=read(base/run_names[0]/'destination-navigation.json')
verification=read(base/run_names[-1]/'place/placement-verification.json')
names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
if (not r7['grasp_success_verified'] or 'carry_destination' not in r7['confirmed_stages']
        or r7['close_commands_sent']!=1 or r10['release_commands_sent']!=1 or not r10['hand_released']
        or any(r['close_commands_sent'] for r in (r10,r11,r12))
        or any(r['release_commands_sent'] for r in (r7,r11,r12))
        or not r12['physical_transfer_verified'] or not r12['placement_success_verified']
        or not r12['robot_ready_for_next_task'] or r12['robot_commands_sent']
        or not robot_ready(dict(zip(names,r12['last_fresh_robot_state']['joints'])))
        or len(r12['final_placement_rows'])!=3 or not verification['placement_success_verified']
        or navigation['status']!='arrived' or not navigation['nav_reset']
        or navigation['current_precision']!=[.06,.08,.06]):
    raise ValueError('actual_staged_transfer_and_final_empty_state_required')

previous=read(base/'ground-pepper-final-observation-ready-progress-20261024.json')
for root in roots:
    if any(sha(root/n)!=d for n,d in previous['latest_expected_hashes'].items()):raise ValueError('tested_source_changed')
archive=base/'pepper-transfer-final-metadata-20261024.tar.gz'
if sha(archive)!=sys.argv[1]:raise ValueError('metadata_patch_changed')
updated={}
with tarfile.open(archive) as tar:
    expected={'OBJECT_TRANSFER.txt','config/manipulation_objects_v1.json'}
    if set(tar.getnames())!=expected:raise ValueError('unexpected_metadata_files')
    for member in tar.getmembers():
        if not member.isfile():raise ValueError('unsafe_archive')
        updated[member.name]=tar.extractfile(member).read()
old_profiles=read(code/'config/manipulation_objects_v1.json')['objects']
new_profiles=json.loads(updated['config/manipulation_objects_v1.json'])['objects']
for name in ('apple','canned_juice','rubiks_cube'):
    if old_profiles[name]!=new_profiles[name]:raise ValueError('other_object_profile_changed')
for key,value in old_profiles['ground_pepper'].items():
    if key not in ('status','note') and new_profiles['ground_pepper'][key]!=value:
        raise ValueError('pepper_motion_or_geometry_profile_changed')

before=Path(r7['close']['before_capture']);snap=read(before/'snapshot.json');measurement=read(before/'object-measurement.json')
preclose=closure_error(measurement['center_m'],snap).tolist()
lift=[r['proof'] for r in r7['lift_verification']]
feedback=read(base/'pepper-raw-stream-timing-20261024-01/result.json')
cfg=read(code/'config/apple_transfer_auto_v1.json');audit=check_inputs(cfg)
with RobotLock(),tarfile.open(base/'pepper-transfer-final-metadata-20261024-previous.tar.gz','x:gz') as backup:
    for root in roots:
        if any(sha(root/n)!=d for n,d in previous['latest_expected_hashes'].items()):raise ValueError('concurrent_edit')
    latest=dict(previous['latest_expected_hashes'])
    for i,root in enumerate(roots):
        for n,content in updated.items():
            backup.add(root/n,arcname=str(i)+'/'+n)
            pending=root/(n+'.verified-metadata-pending')
            with pending.open('xb') as f:f.write(content)
            pending.replace(root/n);latest[n]=hashlib.sha256(content).hexdigest()
    for root in roots:
        if any(sha(root/n)!=d for n,d in latest.items()):raise ValueError('copy_mismatch')
    from manipulation_profiles import object_profile,transfer_config,validate_transfer_config
    object_profile('ground_pepper');validate_transfer_config(transfer_config('ground_pepper'))
    result=dict(version=1,status='ground_pepper_native_source_staged_grasp_carry_place_verified_robot_empty',
        saved_ns=time.time_ns(),class_name='ground_pepper',canonical_class_id=11,
        source='LayoutA bedroom white_side_table#bedroom_1',destination='square_low_table#living_room',
        source_reposition_requested=False,human_object_reposition_required=False,
        default_future_tests_use_native_object_positions=True,
        user_preference_file=str(base/'manipulation-test-preferences.json'),
        actual_staged_runs=[str(base/n) for n in run_names],
        physical_transfer_verified=True,grasp_success_verified=True,carry_success_verified=True,
        placement_success_verified=True,release_commands_sent_total=1,close_commands_sent_total=1,
        cleanup_completed=True,robot_ready_for_next_task=True,robot_holding_pepper=False,
        object_remains_held=False,hand_released=True,
        current_state_supersedes_all_old_held_contexts=True,
        last_verified_robot_state=r12['last_fresh_robot_state'],
        current_state_source=str(base/run_names[-1]/'result.json'),
        last_verified_robot_state_description='EMPTY, open hand, tucked in living room; pepper placed and supported on destination low table',
        historical_state_is_not_new_motion_authorization=True,
        full_uninterrupted_initial_start_transfer_verified=False,
        trial_kind='staged development with actual new-source grasp, actual carry, one release, cleanup, and final read-only proof',
        statistical_success_rate_established=False,
        source_confidences=[row['detection']['confidence'] for row in r7['source_observations']],
        preclose_contact_error_m=preclose,
        actual_small_centering_m=[row['delta'] for row in r7['approaches'] if row['phase']=='small_open_finger_centering'],
        actual_lift_m=[row['object_delta_odom_m'][2] for row in lift],
        object_palm_follow_error_m=[row['object_palm_follow_error_m'] for row in lift],
        afterlift_tilts_deg=[row['measurement']['tilt_deg'] for row in r7['lift_verification']],
        held_opening_rad=r7['held_opening_rad'],
        actual_placement_verification=verification,
        final_placement_captures=[row['capture'] for row in r12['final_placement_rows']],
        final_verification_robot_commands_sent=False,
        minimum_table_outer_clearance_lower_estimate_m=min(row['table_edge_safety']['outer_clearance_lower_estimate_m'] for row in verification['checks']),
        grasp_carry_run_elapsed_s=r7['elapsed_s'],placement_release_run_elapsed_s=r10['elapsed_s'],
        cleanup_run_elapsed_s=r11['elapsed_s'],final_readonly_verification_elapsed_s=r12['elapsed_s'],
        elapsed_values_are_staged_run_durations_not_uninterrupted_task_time=True,
        tests_passed=640,targeted_local_tests_passed=98,
        test_scope='manipulation runtime package; not a claim the whole training workspace is green',
        runtime_test_log=str(base/'pepper-final-observation-20261024-validation/tests.log'),
        metadata_only_update_validated=True,
        model_and_map_audit=audit,model_changed=False,
        navigation_goal_precision_changed=False,navigation_current_precision=navigation['current_precision'],
        navigation_motion_profile=navigation['motion_profile'],navigation_speed_parameters_restored=navigation['speed_parameters_restored'],
        navigation_yaml_or_goal_tolerances_changed=False,
        joint_feedback_max_age_s=.6,rgbd_max_age_s=.4,tf_age_limit_changed=False,
        measured_raw_joint_age_s=feedback['age_ranges_s']['joints'],
        joint_latency_policy_is_class_specific=True,original_can_default_joint_age_unchanged=True,
        original_timestamps_rewritten=False,underlying_clock_or_transport_latency_cause_not_resolved=True,
        extra_seat_m=0.,finger_midline_base_y_offset_m=-.00005,lateral_alignment_tolerance_m=.004,
        task_finished_sent=False,latest_expected_hashes=latest,
        next=[
            'Do not resume any old held-pepper receipt: the actual final state is EMPTY.',
            'Inspect current object locations before another task; do not request default human relocation.',
            'The fixed-route ground-pepper primitives succeeded as a staged chain; a single uninterrupted initial-start regression remains pending.',
            'Keep 4mm centering, zero extra forward seating, bounded 600ms pepper joint feedback, unchanged images/TF, and current navigation settings.',
            'Repeated trials and more object classes are still required before claiming statistical manipulation reliability.'
        ])
    filename='ground-pepper-native-transfer-success-progress-20261024.json'
    content=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    for p in (base/filename,repo/'training'/filename):
        with p.open('x') as f:f.write(content)
    print(json.dumps({k:result[k] for k in ('status','physical_transfer_verified','robot_ready_for_next_task',
        'robot_holding_pepper','close_commands_sent_total','release_commands_sent_total',
        'minimum_table_outer_clearance_lower_estimate_m','preclose_contact_error_m','tests_passed',
        'full_uninterrupted_initial_start_transfer_verified')},ensure_ascii=False),flush=True)
