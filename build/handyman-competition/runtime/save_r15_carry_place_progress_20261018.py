"""Audit actual terminal carry/place evidence and sync dated progress, no motion."""
import hashlib
import json
import math
from pathlib import Path
import sys
import tarfile
import time
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
from apple_transfer_auto_plan import map_pose
def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
name='apple-carry-place-live-progress-20261018.json';pointer='D:/handyman-training-runs/'+name
previous={
    'apple-close-lift-live-progress-20261018.json':'5fca9af502a31361dcbaf9263fbd089e18cd0ee62ca2d8899e5dd3192e4ccd3a',
    'apple-capture-budget-live-progress-20261018.json':'65f7f7079f66ea7e876103df2a0223200d1bda4d94a501305bd9d167aca9b249',
    'apple-near-geometry-live-progress-20261018.json':'469bb8f9e4709c37d0ba49a0c354a920f52dcbff235c29a32d8d5b112609e5ac',
    'apple-near-geometry-progress-20261018.json':'e3fbe0a401f5e8681edc2bb833233231635d086a0d231e4c5a38b63ac7245f36',
    'phase5-restart-checkpoint-20261017.json':'4e24e4006230ed17ed1ea9b3b1fcd0559906bf17a4037fbccce4ac65dfb85861',
    'apple-transfer-automation-progress-20261017.json':'d68a746a48d899a0b075b76e2f1e2fbd4f7826b57995425bc8507221d28375f9',
    'apple-transfer-live-progress-20261017.json':'563a35948f194908dc9d659dda3cde8d83a5400b1b14b19e559a81d226d7d4db'}
roots=dict(runs=base,runtime=code,training=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training'),
    module=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/automation/apple_transfer_v1'))
with RobotLock():
    for root in roots.values():
        if (root/name).exists():raise ValueError('new_progress_file_required')
        for old,digest in previous.items():
            if sha(root/old)!=digest:raise ValueError('progress_changed_since_previous_sync:'+str(root/old))
    run=base/'apple-r15-reviewed-carry-place-20261018-01';result=read(run/'result.json')
    if result['status']=='running':raise ValueError('terminal_actual_state_required')
    success=result['status']=='reviewed_r15_carry_placement_verified'
    verification=read(run/'placement-verification.json') if (run/'placement-verification.json').exists() else None
    sequence=read(run/'placement-approach-sequence.json') if (run/'placement-approach-sequence.json').exists() else None
    navigation=read(run/'destination-navigation.json') if (run/'destination-navigation.json').exists() else None
    post=read(base/'apple-r15-post-carry-place-20261018-01/result.json');capture=Path(post['last_capture']);snapshot=read(capture/'snapshot.json');j=snapshot['joints']
    ready=post['readiness']
    if success:
        if (verification.get('placement_success_verified') is not True or verification.get('table_edge_safety_verified') is not True
            or verification.get('frames',0)<3 or ready.get('ready') is not True
            or navigation.get('nav_reset') is not True or navigation.get('speed_parameters_restored') is not True
            or navigation.get('goal_precision_changed') is not False
            or sequence.get('approach_mode')!='single' or len(sequence.get('steps',[]))!=1):raise ValueError('independent_completion_audit_failed')
    arm=[j[k] for k in ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint')]
    robot=dict(capture=str(capture),depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        holding_object=False if ready.get('ready') is True else 'requires_current_visual_audit',
        arm_is_tucked=all(abs(a-b)<.005 for a,b in zip(arm,[0,0,0,-math.pi/2,0])),arm=arm,
        hand_motor=j['hand_motor_joint'],head=[j[k] for k in ('head_pan_joint','head_tilt_joint')],base_map_pose=map_pose(snapshot),
        room='living_room' if success else 'read actual pose/stage',
        apple_location='Verified on living_room square_low_table; no longer held or on bedroom source table' if success else 'Read exact stop evidence; do not assume held or placed',
        current_postrun_empty_tucked_readiness=ready.get('ready'),fresh_state_required_before_next_action=True,
        command_publishers=ready.get('command_publishers'),controller_state=ready.get('controller_state'),
        warning='Last-confirmed physical state, not a future live action/identity lease; never replay an old trajectory or numeric map anchor after restart.')
    next_step=('Read this checkpoint FIRST. CURRENT EMPTY/OPEN/TUCKED in living_room; apple ALREADY PLACED on square_low_table. Do not repeat close/lift/carry/release. Next is a NEW uninterrupted full-entry regression from genuinely restored initial Session1/LayoutA/source-apple state, with fresh source synchronization/localization/readiness. The completed reviewed continuation is NOT the previously stopped full r13c run becoming successful.' if success else
        'Read exact current result/stage and NEW post-stop captures FIRST. No automatic failed-action retry, source replay, release, or empty-tucked entry until actual grip/arm/object/location are freshly audited.')
    progress=dict(status='reviewed_r15_carry_and_edge_safe_placement_verified' if success else 'reviewed_r15_carry_place_stopped',
        branch='feat/handyman-phase5-grasping',recorded_ns=time.time_ns(),revision='Existing r15/r14/r13c automation; no production code change this carry test',
        current_robot=robot,actual_continuation=dict(directory=str(run),result=result),
        source_close_lift_checkpoint='D:/handyman-training-runs/apple-close-lift-live-progress-20261018.json',
        navigation=navigation,
        placement=dict(verified=success,destination='square_low_table#living_room',
            approach_commands=len(sequence['steps']) if sequence else None,
            approach_translation_m=sequence['total_translation_m'] if sequence else None,
            actual_steps=sequence.get('steps') if sequence else None,
            final_verification=verification,
            minimum_final_outer_clearance_estimate_m=min(row['table_edge_safety']['outer_clearance_lower_estimate_m'] for row in verification['rows']) if success else None,
            actual_release_evidence=read(run/'place-release/result.json').get('release_geometry_evidence') if (run/'place-release/result.json').exists() else None,
            edge_policy=dict(minimum_outer_gap_m=.03,uncertainty_allowance_m=.01),single_translation_cap=None),
        post_run_audit=post,production_code_model_maps_or_unity_files_changed=False,
        map_anchor_changed_this_turn=False,current_map_anchor_pid_last_observed=6862,
        navigation_goal_tolerances_changed=False,navigation_speed_profile='balanced_v1; original limits restored after navigation',
        prior_close_or_lift_or_source_approach_replayed=False,robot_commands_sent_this_turn=True,
        task_finished_sent=False,does_not_exist_sent=False,git_commit_or_push_performed=False,
        whole_uninterrupted_transfer_verified=False,
        scope='One reviewed live carry/place continuation from a separately verified apple grasp. Not an initial-pose uninterrupted full transfer, generic30-object executor or competition success-rate measurement.',next=next_step)
    updates={name:progress}
    for old in previous:
        value=read(base/old);value.update(latest_validation_checkpoint=pointer,latest_live_validation_checkpoint=pointer,
            current_state_authoritative_checkpoint=pointer,latest_carry_placement_status=progress['status'],next=next_step,
            older_current_fields_are_historical=True)
        if old in ('phase5-restart-checkpoint-20261017.json','apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json'):
            value.update(current_robot=robot,latest_robot=robot,latest_capture=str(capture),current=progress['status'])
            if success:value['latest_live_placement_verification']=dict(file=str(run/'placement-verification.json'),placement_success_verified=True,table_edge_safety_verified=True,
                frames=verification['frames'],span_s=verification['span_s'],estimated_outer_clearance_m=progress['placement']['minimum_final_outer_clearance_estimate_m'],
                robot_tucked_and_gripper_open=True,no_intervention_full_transfer_verified=False)
        if old=='phase5-restart-checkpoint-20261017.json':
            value.update(navigation_enabled=False,navigation_state='Current carry/place terminal; Nav2 reset, speed limits restored. Read canonical current state.')
        updates[old]=value
    backup=base/'apple-before-r15-carry-place-live-20261018-checkpoints.tar.gz'
    with tarfile.open(backup,'x:gz') as archive:
        for label,root in roots.items():
            for old in previous:archive.add(root/old,arcname=label+'/'+old)
    for root in roots.values():
        for filename,value in updates.items():(root/filename).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    hashes={filename:sha(base/filename) for filename in updates}
    if any(sha(root/filename)!=digest for root in roots.values() for filename,digest in hashes.items()):raise ValueError('metadata_sync_hash_mismatch')
    report=dict(status='r15_carry_place_progress_synced',copies=4,hashes=hashes,backup=str(backup),placement_verified=success,
        current_capture=str(capture),robot_commands_sent=False)
    with (base/'apple-r15-carry-place-live-20261018-checkpoint-sync.json').open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report),flush=True)
