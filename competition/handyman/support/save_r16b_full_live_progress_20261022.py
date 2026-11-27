"""Save completed NEW full-run evidence and current terminal state; no motion."""
import hashlib
import json
import math
from pathlib import Path
import sys
import tarfile
import time

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,STAGES,write_json
from apple_transfer_auto_plan import map_pose
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
roots=dict(runs=base,runtime=code,training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
name='apple-r16b-full-live-progress-20261022.json';pointer='D:/handyman-training-runs/'+name
previous={
    'phase5-visual-test-preparation-20261022.json':'0f4dd986f8c0f9fc34b0752a57393face79e029ca3eacb299a0b371c8ff1865f',
    'phase5-restart-checkpoint-20261017.json':'590bbe2eacb99e322e3ce7ca71390215d516e3349fca98e35f7051bcb1defa7c',
    'apple-transfer-automation-progress-20261017.json':'9d68b20af3ef461f0feaf0a269be8461576a8dfdefefcdd4e928f711e5e5c270',
    'apple-transfer-live-progress-20261017.json':'7589228b80c914336fc0fa4e03b425dc8c29907b3717faba38236625dd8e4a42',
    'apple-outcome-policy-progress-20261018.json':'f3ad3b2c2ab2d9eb67daf31a596584f8cc2c4cb1a278900a1fd1b8f9ff094126'}
trial=base/'automatic-apple-r16b-20261022-01'
with RobotLock():
    for root in roots.values():
        if (root/name).exists():raise ValueError('new_progress_checkpoint_required')
        for old,digest in previous.items():
            if sha(root/old)!=digest:raise ValueError('progress_changed:'+str(root/old))
    result=read(trial/'result.json');verification=read(trial/'placement-verification.json')
    grasp=read(trial/'grasp/close-lift/result.json');lift=read(trial/'grasp/close-lift/independent-lift-verification.json')
    post=read(base/'apple-r16b-post-full-live-20261022-01/result.json')
    source=read(trial/'source-navigation.json');destination=read(trial/'destination-navigation.json')
    approach=read(trial/'placement-approach-sequence.json')
    if (result['status']!='automatic_transfer_verified' or result['run_kind']!='full_transfer'
            or result['confirmed_stages']!=list(STAGES) or not result['task_objective_completed']
            or not verification['placement_success_verified'] or not verification['table_edge_safety_verified']
            or not lift['grasp_success_verified'] or not post['readiness']['ready']
            or result['automatic_action_retry'] or result['automatic_resume']
            or any(n.get('status')!='arrived' or not n.get('nav_reset') or not n.get('speed_parameters_restored')
                   or n.get('goal_precision_changed') for n in (source,destination))):
        raise ValueError('actual_full_transfer_and_terminal_state_required')
    close_commands=[e for e in grasp['close']['events'] if e.get('event')=='command' and e.get('kind')=='hand']
    release=read(trial/'place-release/result.json')
    release_commands=[e for e in release['events'] if e.get('event')=='command' and e.get('kind')=='hand']
    if len(close_commands)!=1 or len(release_commands)!=1 or len(approach['steps'])!=1:
        raise ValueError('single_close_release_and_placement_approach_required')
    capture=Path(post['last_capture']);snapshot=read(capture/'snapshot.json');j=snapshot['joints']
    events=[json.loads(line) for line in (trial/'journal.jsonl').read_text().splitlines()]
    seconds=(events[-1]['time_ns']-events[0]['time_ns'])/1e9
    robot=dict(capture=str(capture),depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        holding_object=False,arm_is_tucked=True,arm=[j[k] for k in ('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint')],
        hand_motor=j['hand_motor_joint'],head=[j[k] for k in ('head_pan_joint','head_tilt_joint')],base_map_pose=map_pose(snapshot),
        room='living_room',apple_location='Verified placed on square_low_table#living_room; source white_side_table no longer contains the transferred apple',
        command_publishers=post['readiness']['command_publishers'],controller_state=post['readiness']['controller_state'],
        fresh_state_required_before_next_action=True,warning='Last-confirmed terminal state, not a future action/identity lease.')
    deployment=read(base/'apple-outcome-r16b-20261018-deployment.json')
    for label,root in roots.items():
        if label!='runs' and any(sha(root/file)!=digest for file,digest in deployment['hashes'].items()):raise ValueError('runtime_code_changed')
    next_step=('Read this checkpoint FIRST. One NEW uninterrupted full-entry LayoutA apple transfer completed all17 stages. '
        'Current last-confirmed state EMPTY/OPEN/TUCKED in living_room; apple ALREADY ON square_low_table. '
        'No automatic retry/resume, second closure/release or old action replay. Do not rerun bedroom-source entry unless user genuinely restores source apple/initial state. '
        'RViz remains open. The current scene map anchor was newly initialized from configured spawn and fresh laser; do not reuse numeric anchor after Unity restart. '
        'For another trial, freshly inspect actual RGBD/arm/grip/localization. This is one apple trial, not30-object generalization or80/85percent success-rate acceptance. '
        'Competition task parser/Moderator integration remains outside this dedicated transfer profile; no Task_finished was sent. '
        'Cold idle Nav2 requires configure-then-cleanup, without activation or setting parameters, before reading plugin tolerance values; see controller-initialization.json.')
    progress=dict(status='new_uninterrupted_full_apple_transfer_verified',revision='outcome-r16b',saved_ns=time.time_ns(),
        branch='feat/handyman-phase5-grasping',wsl_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        trial_directory=str(trial),task_id=result['task_id'],automatic_result=result,whole_uninterrupted_transfer_verified=True,
        successful_full_trials_this_run=1,execution_duration_s=seconds,confirmed_stage_count=len(result['confirmed_stages']),
        current_robot=robot,post_run_readiness=post['readiness'],latest_capture=str(capture),
        grasp=dict(success_verified=True,closure_commands=1,verification_file=str(trial/'grasp/close-lift/independent-lift-verification.json'),
            verified_follow_frames=lift['accepted_after_frames'],verified_follow_span_s=lift['after_span_s'],
            canonical_detector_verdict_preserved=grasp['grasp_success_verified'],effective_rgbd_follow_verification=True),
        placement=dict(success_verified=True,destination='square_low_table#living_room',release_commands=1,
            single_approach_commands=1,single_approach_distance_m=approach['total_translation_m'],
            verified_frames=verification['frames'],verified_span_s=verification['span_s'],center_spread_m=verification['center_spread_m'],
            minimum_outer_edge_clearance_estimate_m=min(row['table_edge_safety']['outer_clearance_lower_estimate_m'] for row in verification['rows']),
            cleanup_required=verification['cleanup_required'],verification_file=str(trial/'placement-verification.json')),
        navigation=dict(source_position_error_m=source['position_error_m'],destination_position_error_m=destination['position_error_m'],
            tolerances=[.06,.08,.06],goal_precision_changed=False,speed_profile='balanced_v1',original_speed_limits_restored=True),
        initial_preparation=str(base/'apple-r16b-live-initial-20261022-01'),
        robot_commands_sent_this_trial=True,additional_robot_commands_sent_by_checkpoint=False,
        production_code_model_maps_unity_or_navigation_goal_tolerances_changed=False,
        task_finished_sent=False,does_not_exist_sent=False,automatic_retry=False,automatic_resume=False,
        manual_motion_interventions_during_run=False,git_commit_or_push_performed=False,
        scope='One dedicated simulated apple transfer from fresh initial scene, not generic task/competition integration or statistical success-rate acceptance.',next=next_step)
    updates={name:progress}
    for old in previous:
        value=read(base/old);value.update(latest_full_live_validation_checkpoint=pointer,latest_live_validation_checkpoint=pointer,
            current_state_authoritative_checkpoint=pointer,latest_validation_checkpoint=pointer,
            latest_full_live_status=progress['status'],older_current_fields_are_historical=True,next=next_step)
        if old in ('phase5-restart-checkpoint-20261017.json','apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json'):
            value.update(current_robot=robot,latest_robot=robot,latest_capture=str(capture),current=progress['status'])
        if old=='apple-outcome-policy-progress-20261018.json':
            value.update(last_confirmed_robot=robot,new_live_trial_performed=True,whole_uninterrupted_transfer_verified=True,
                live_validation_scope='One dedicated LayoutA apple full transfer; no statistical success-rate claim.')
        updates[old]=value
    backup=base/'apple-before-r16b-full-live-20261022-checkpoints.tar.gz'
    with tarfile.open(backup,'x:gz') as archive:
        for label,root in roots.items():
            for old in previous:archive.add(root/old,arcname=label+'/'+old)
    for root in roots.values():
        for filename,value in updates.items():write_json(root/filename,value)
    hashes={filename:sha(base/filename) for filename in updates}
    if any(sha(root/filename)!=digest for root in roots.values() for filename,digest in hashes.items()):raise ValueError('metadata_sync_mismatch')
    report=dict(status='full_r16b_live_progress_synced',copies=4,hashes=hashes,backup=str(backup),
        whole_uninterrupted_transfer_verified=True,duration_s=seconds,robot_commands_sent=False)
    with (base/'apple-r16b-full-live-20261022-checkpoint-sync.json').open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report),flush=True)
