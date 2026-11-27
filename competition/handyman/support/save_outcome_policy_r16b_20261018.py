"""Persist acceptance-policy handoff; never turn recorded state into live authority."""
import hashlib
import json
from pathlib import Path
import sys
import tarfile
import time

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
from task_outcome_policy import (MIN_VERIFICATION_LIFT_M,MAX_PALM_FOLLOW_ERROR_M,
    MAX_HOLD_OPENING_DRIFT_RAD,RELEASE_BOTTOM_GAP_M,PLACED_BOTTOM_GAP_M,
    PLACED_CENTER_SPREAD_M,OPEN_HAND_MIN_RAD)
read=lambda path:json.loads(path.read_text())
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
roots=dict(runs=base,runtime=code,training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
name='apple-outcome-policy-progress-20261018.json';pointer='D:/handyman-training-runs/'+name
previous={
    'apple-carry-place-live-progress-20261018.json':'74c6a4c217512ed16c215c88a1f04a416ca2981a62a0b69644a3fd2396685211',
    'phase5-restart-checkpoint-20261017.json':'124beaba3edd75a99cc80f06414a94ca4921fcb16e9421d8c3a3906a1b7bca3e',
    'apple-transfer-automation-progress-20261017.json':'c117d8311acfbf3de54a32425c3fb41b44ecded05ea4f282c70bde0ffd598246',
    'apple-transfer-live-progress-20261017.json':'6690059817cbde317b2ede3cc501c81135eac4acc7fbee8d43a3369be358f226'}
with RobotLock():
    deployment=read(base/'apple-outcome-r16b-20261018-deployment.json')
    regression=read(Path(deployment['recorded_regression']))
    if (deployment['status']!='deployed_tests_passed' or regression['status']!='recorded_outcome_regression_passed'
            or deployment['runtime_tests']['count']!=367 or deployment['robot_commands_sent'] is not False):
        raise ValueError('actual_deployment_and_regression_required')
    for label,root in roots.items():
        if (root/name).exists():raise ValueError('new_policy_checkpoint_required')
        for old,digest in previous.items():
            if sha(root/old)!=digest:raise ValueError('metadata_changed:'+str(root/old))
        if label!='runs' and any(sha(root/file)!=digest for file,digest in deployment['hashes'].items()):
            raise ValueError('runtime_code_changed_since_deployment')
    physical=read(base/'apple-carry-place-live-progress-20261018.json')
    next_step=('Read latest_policy_checkpoint BEFORE the r15 physical-state checkpoint. r16b uses actual grasp/place outcomes, '
        'not exact planned XY/10cm/historical joint poses. No robot commands were sent for this policy change. '
        'Last-confirmed state only: EMPTY/OPEN/TUCKED in living_room; apple ALREADY ON square_low_table. '
        'Do not replay close/lift/carry/release. A new full regression requires genuinely restored Session1/LayoutA/source-apple state '
        'and fresh localization/readiness; never blindly restore the old numeric map anchor after Unity restart. '
        'Placement success is distinct from robot cleanup readiness; cleanup_required must not replay release or silently start another task.')
    progress=dict(status='outcome_policy_deployed_offline_and_recorded_regressions_passed',revision='outcome-r16b',
        saved_ns=time.time_ns(),branch='feat/handyman-phase5-grasping',
        deployment='D:/handyman-training-runs/apple-outcome-r16b-20261018-deployment.json',
        recorded_regression='D:/handyman-training-runs/apple-r16b-recorded-outcome-regression-20261018-01.json',
        runtime_path='D:/handyman-tools/apple-transfer-auto-v1',runtime_tests=deployment['runtime_tests'],
        policy_file='D:/handyman-tools/apple-transfer-auto-v1/task_outcome_policy.py',
        changed_files=list(deployment['hashes']),code_hashes=deployment['hashes'],
        removed_or_replaced=['planned release XY <=12mm','uniform exact historical release joints <=0.003',
            'exact10cm lift /0.5mm endpoint frame selection','absolute static object pose during successful measured palm-following',
            'single-placement odometry <=8mm as final task gate','exact tucked arm and0.65hand as object-placement proof',
            'withdrawal cleanup failure negating independently observed placement','carry/cleanup replaying nominal lift instead of actual state'],
        acceptance_allowances=dict(minimum_measured_lift_m=MIN_VERIFICATION_LIFT_M,
            palm_follow_max_error_m=MAX_PALM_FOLLOW_ERROR_M,holding_opening_drift_rad=MAX_HOLD_OPENING_DRIFT_RAD,
            release_bottom_gap_m=list(RELEASE_BOTTOM_GAP_M),placed_bottom_gap_m=PLACED_BOTTOM_GAP_M,
            placed_center_spread_m=PLACED_CENTER_SPREAD_M,hand_open_min_rad=OPEN_HAND_MIN_RAD),
        unchanged=dict(navigation_tolerances=[.06,.08,.06],motion_profile='balanced_v1',
            single_placement_translation_cap=None,table_outer_edge_gap_m=.03,table_uncertainty_allowance_m=.01,
            rgbd_freshness_s=.4,object_identity_and_destination_checks=True,control_ownership_checks=True,
            model_maps_unity_unchanged=True,automatic_motion_retry=False),
        latest_live_validation_checkpoint='D:/handyman-training-runs/apple-carry-place-live-progress-20261018.json',
        last_confirmed_robot=physical['current_robot'],last_confirmed_state_is_not_current_live_lease=True,
        original_trial_results_unchanged=True,new_live_trial_performed=False,robot_commands_sent=False,
        whole_uninterrupted_transfer_verified=False,task_finished_sent=False,git_commit_or_push_performed=False,
        scope='Reviewed LayoutA apple transfer only; not generic30-object completion, competition protocol or success-rate acceptance.',next=next_step)
    updates={name:progress}
    for old in previous:
        value=read(base/old)
        value.update(latest_policy_checkpoint=pointer,latest_code_revision='outcome-r16b',next=next_step,
            historical_trial_thresholds_do_not_override_latest_policy=True)
        updates[old]=value
    backup=base/'apple-before-outcome-r16b-20261018-checkpoints.tar.gz'
    with tarfile.open(backup,'x:gz') as archive:
        for label,root in roots.items():
            for old in previous:archive.add(root/old,arcname=label+'/'+old)
    for root in roots.values():
        for filename,value in updates.items():(root/filename).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    hashes={filename:sha(base/filename) for filename in updates}
    if any(sha(root/filename)!=digest for root in roots.values() for filename,digest in hashes.items()):
        raise ValueError('metadata_sync_hash_mismatch')
    result=dict(status='outcome_r16b_policy_progress_synced',copies=4,hashes=hashes,backup=str(backup),
        physical_state_reobserved=False,robot_commands_sent=False)
    with (base/'apple-outcome-r16b-20261018-checkpoint-sync.json').open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result),flush=True)
