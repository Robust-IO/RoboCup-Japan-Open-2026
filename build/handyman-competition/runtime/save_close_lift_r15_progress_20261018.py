"""Preserve original trial, write authoritative holding-state progress to4 copies."""
import hashlib
import json
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
new='apple-close-lift-live-progress-20261018.json';pointer='D:/handyman-training-runs/'+new
previous={
    'apple-capture-budget-live-progress-20261018.json':'287f9de07fb86b4eafa0db336845295de6689c9763480e80c1cf1a7569c5a19a',
    'apple-near-geometry-live-progress-20261018.json':'05aaaadba4d6c14050b0ae7fa8f99e4dba6f7383f74f8a935cae1edac3c800b7',
    'apple-near-geometry-progress-20261018.json':'e49ab708a8afa9bc730e26988e22b2208cac9b61658083b5c494f857b296ac57',
    'phase5-restart-checkpoint-20261017.json':'13f8b0fd90df0ddaebab4127c0f27415491d5194b1ae890237a2e7ea8186c498',
    'apple-transfer-automation-progress-20261017.json':'3eb9556d9223d615113593a9a2c641368638fa2325be95dd29d3ad66126992fa',
    'apple-transfer-live-progress-20261017.json':'ad92bf7d198d79b713d661762dde6ddcab416725e7d07c14c343fa18f21130cf'}
roots=dict(runs=base,runtime=code,training=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training'),
    module=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/automation/apple_transfer_v1'))
with RobotLock():
    for root in roots.values():
        if (root/new).exists():raise ValueError('new_checkpoint_required')
        for name,digest in previous.items():
            if sha(root/name)!=digest:raise ValueError('previous_progress_changed:'+str(root/name))
    attempt=base/'apple-r14-reviewed-close-lift-20261018-01'
    trial=read(attempt/'result.json');actual=read(attempt/'grasp/close-lift/result.json')
    audit_path=base/'apple-r15-r14-settled-lift-reaudit-20261018-01.json';reaudit=read(audit_path);proof=reaudit['independent']
    deployed=read(base/'apple-lift-settled-r15-20261018-deployment.json')
    if (deployed['status']!='deployed_tests_passed' or reaudit['status']!='recorded_lift_settled_window_verified'
        or proof['grasp_success_verified'] is not True or reaudit['original_files_unchanged'] is not True):raise ValueError('tested_verified_evidence_required')
    post=read(base/'apple-r14-post-close-lift-20261018-01/result.json');capture=Path(post['last_capture']);snapshot=read(capture/'snapshot.json');j=snapshot['joints']
    opening=actual['close']['closure_evidence']['observed_opening_rad']
    if trial['base_approach_commands_sent']!=0 or len(actual['commands'])!=2:raise ValueError('one_close_one_lift_no_base_approach_required')
    if abs(j['hand_motor_joint']-opening)>.005 or abs(j['arm_lift_joint']-actual['lift']['final_joints'][0])>.005:raise ValueError('post_run_holding_posture_changed')
    robot=dict(capture=str(capture),depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        holding_object=True,object='apple',arm_is_tucked=False,
        arm=[j[k] for k in ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint')],
        hand_motor=j['hand_motor_joint'],head=[j[k] for k in ('head_pan_joint','head_tilt_joint')],
        base_map_pose=map_pose(snapshot),room='bedroom',
        apple_location='Held above source white_side_table after one verified10cm lift; NOT still on table and NOT placed on square_low_table',
        fresh_state_required_before_next_action=True,
        warning='Historical last-confirmed state, not a reusable live identity/action lease. Do not reopen hand or run empty-tucked/source-grasp entry.')
    progress=dict(status='reviewed_single_close_and_10cm_lift_verified_with_corrected_settled_window',
        branch='feat/handyman-phase5-grasping',recorded_ns=time.time_ns(),revision='lift-settled-r15 over close-handoff-r14/capture-r13c',
        current_robot=robot,current_map_anchor_pid_last_observed=6862,
        actual_trial=dict(directory=str(attempt),task_id=trial['task_id'],epoch=trial['epoch'],
            original_report_status=trial['status'],original_result_preserved=True,
            new_classified_identity_then_near_geometry=True,new_readonly_close_samples=len(trial['readonly_samples']),
            base_approach_commands_sent=0,gripper_close_commands=1,lift_commands=1,lift_distance_m=.1,
            observed_contact_opening_rad=opening,contact_stable_samples=actual['close']['closure_evidence']['stable_samples'],
            contact_stable_duration_s=actual['close']['closure_evidence']['stable_duration_s'],
            close_publish_observation_mode=actual['close']['closure_identity_evidence']['observation_mode'],
            old_actions_or_identity_replayed=False,holding_retraction_sent=False,destination_navigation_sent=False,release_sent=False),
        r14_live_handoff=dict(corrected_runner_reached_close_without_any_additional_base_approach=True,
            same_live_recording_used_for_publication_lease_and_actual_contact_and_lift=True),
        r15_fix=dict(reason='Latest joint reached endpoint while original head image was still from moving lift; transition image inflated all-frame spread to11.49mm',
            completed_at_ns=actual['lift_completed_at_ns'],frozen_window_end_ns=actual['verification_window_end_ns'],
            rule='All original RGB/depth/hand stamps must be in actual post-completion/frozen verification window. All frames selected by phase retain old checks.',
            stability_limit_unchanged_m=.01,minimum_frames_unchanged=3,minimum_span_unchanged_s=.4,
            original_source_stamps_preserved=True,no_threshold_widening=True,no_motion_replay=True),
        corrected_recorded_verification=dict(file=str(audit_path),grasp_success_verified=True,
            frames=proof['accepted_after_frames'],span_s=proof['after_span_s'],center_spread_m=proof['after_center_spread_m'],
            excluded_transition=proof['excluded_transition_captures'],settled_rejected=proof['rejected'],
            all_original_files_unchanged=True,canonical_detector_only_status=reaudit['canonical_detector_only']['status'],
            current_live_motion_permission=False),
        deployment=deployed,post_run_audit=post,
        robot_commands_sent_this_turn=True,grasp_verified=True,whole_uninterrupted_transfer_verified=False,
        navigation_parameters_or_map_anchor_or_model_or_unity_files_changed_this_turn=False,
        task_finished_sent=False,does_not_exist_sent=False,git_commit_or_push_performed=False,
        scope='One newly identified simulated LayoutA apple close/lift from prior prepared pose. Not a successful uninterrupted full run, placement,30-object generalization or competition success rate.',
        next='Read this checkpoint FIRST. Current APPLE HELD / ARM EXTENDED / bedroom. Freshly verify grip/arm/TF and apple geometry, then KEEP grip while retracting for carry. Use reviewed bedroom→living_room route and unchanged navigation tolerances, then measured table height/inverse placement to square_low_table. Do NOT repeat source approach, close/lift, old commands, open hand, or launch empty-tucked full entry. Original r14 reports remain inconclusive; explicitly bind the separate corrected r15 audit for any reviewed continuation. New full r15 automation still requires a genuinely new initial-state regression.')
    updates={new:progress}
    for name in previous:
        value=read(base/name);value.update(latest_validation_checkpoint=pointer,latest_live_validation_checkpoint=pointer,
            current_state_authoritative_checkpoint=pointer,r15_latest_status=progress['status'],next=progress['next'],
            older_current_fields_are_historical=True)
        if name in ('phase5-restart-checkpoint-20261017.json','apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json'):
            value.update(current_robot=robot,latest_robot=robot,latest_capture=str(capture),current=progress['status'])
        if name=='phase5-restart-checkpoint-20261017.json':
            value.update(navigation_enabled=False,navigation_state='Idle after prior source route. Current APPLE HELD, arm extended after verified close/lift; no new navigation this turn.')
        updates[name]=value
    backup=base/'apple-before-close-lift-live-r15-20261018-checkpoints.tar.gz'
    with tarfile.open(backup,'x:gz') as archive:
        for label,root in roots.items():
            for name in previous:archive.add(root/name,arcname=label+'/'+name)
    for root in roots.values():
        for name,value in updates.items():(root/name).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    hashes={name:sha(base/name) for name in updates}
    if any(sha(root/name)!=digest for root in roots.values() for name,digest in hashes.items()):raise ValueError('progress_copy_mismatch')
    report=dict(status='close_lift_holding_checkpoints_synced',copies=4,hashes=hashes,backup=str(backup),
        holding_object=True,arm_extended=True,robot_commands_sent=False)
    with (base/'apple-close-lift-live-r15-20261018-checkpoint-sync.json').open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report),flush=True)
