"""Preserve the failed trial and synchronize its fresh terminal checkpoint only."""
import hashlib
import json
import math
from pathlib import Path
import sys
import tarfile
import time

BASE = Path('/__HANDYMAN_PACKAGE__/runtime')
CODE = Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
TRIAL = BASE / 'automatic-apple-r18b-20261022-01'
OUT = BASE / 'apple-r18b-post-stopped-20261022-01'
sys.path.insert(0, str(CODE))
from apple_transfer_automation import RobotLock, write_json

read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
name = 'apple-r18b-full-live-progress-20261022.json'
roots = dict(runs=BASE, runtime=CODE,
             training=Path('/__HANDYMAN_PACKAGE__/project/training'),
             module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
pointers = ['phase5-visual-test-preparation-20261022.json', 'phase5-restart-checkpoint-20261017.json',
            'apple-transfer-automation-progress-20261017.json', 'apple-transfer-live-progress-20261017.json',
            'apple-outcome-policy-progress-20261018.json', 'apple-efficiency-r17-progress-20261022.json',
            'apple-terminal-r18-progress-20261022.json']
with RobotLock():
    if any((root / name).exists() for root in roots.values()):
        raise ValueError('new_checkpoint_required')
    result = read(TRIAL / 'result.json')
    post = read(OUT / 'result.json')
    grasp = read(TRIAL / 'grasp/result.json')
    lift = read(TRIAL / 'grasp/close-lift/independent-lift-verification.json')
    if result['status'] != 'stopped' or result['stage'] != 'retract_holding' or not lift['grasp_success_verified']:
        raise ValueError('specific_stopped_trial_required')
    if any(post['command_publishers'].values()):
        raise ValueError('unexpected_motion_owner_after_stop')
    hashes = {}
    for filename in ('apple-efficiency-r17-20261022-deployment.json',
                     'apple-terminal-r18-20261022-deployment.json',
                     'apple-terminal-r18b-20261022-deployment.json'):
        hashes.update(read(BASE / filename)['hashes'])
    if any(sha(CODE / n) != d for n, d in hashes.items()):
        raise ValueError('production_code_changed_during_trial')
    events = [json.loads(line) for line in (TRIAL / 'journal.jsonl').read_text().splitlines()]
    starts = {}
    stages = {}
    for event in events:
        if event['event'] == 'intent':
            starts[event['stage']] = event['time_ns']
        elif event['event'] == 'confirmed':
            stages[event['stage']] = (event['time_ns']-starts[event['stage']]) / 1e9
    duration = (events[-1]['time_ns']-events[0]['time_ns']) / 1e9
    robot = dict(state='ARM_RETRACTED_HIGH_HOLDING_APPLE_CARRY_NOT_CONFIRMED',
                 holding_object=True, holding_evidence='Independent RGBD lift-follow passed before retraction; post-stop hand.png shows apple; hand opening unchanged.',
                 arm_is_tucked=True, carry_ready=False, joints=post['current_joints'],
                 base_map_pose=post['current_base_map_pose'], room='bedroom',
                 capture=post['current_capture'], command_publishers=post['command_publishers'],
                 apple_location='In gripper, not on source table and not delivered to square_low_table.',
                 fresh_state_required_before_next_action=True,
                 last_state_is_not_future_motion_lease=True)
    instruction = ('Read this NEW checkpoint FIRST. The r18b full timed trial STOPPED at203.497s during retract_holding, '
        'NOT a completed transfer time. One terminal correction succeeded, one closure and lift were RGBD-verified. '
        'Coordinated wrist/flex retraction was commanded once; base_moved halted the action before carry-ready confirmation. '
        'Post-stop actual arm is flex0/wrist-pi/2/lift0.354958, hand0.382302, apple visible in hand camera. '
        'Current measured offset from retract anchor: translation1.319mm, yaw0.036459rad (2.089deg); '
        'the source guard is translation15mm OR yaw0.03rad (1.719deg). No exact offending pose was saved by the child. '
        'No destination navigation, release or placement. No automatic retry/resume, no code/parameter changes during test. '
        'All command publishers zero, RViz left unchanged. Do not rerun source grasp or reuse old numeric coordinates. '
        'Next action requires fresh actual holding/arm/base/localization audit. If Unity restarted, initialize/check fresh scene. '
        'Old r17 empty/extended and r16b empty/living states are historical. Diagnosis does not establish collision or physics cause.')
    progress = dict(status='r18b_full_timed_trial_stopped_during_carry_retraction',
                    revision='terminal-correction-r18b', saved_ns=time.time_ns(),
                    branch='feat/handyman-phase5-grasping',
                    wsl_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                    task_id=result['task_id'], trial_directory=str(TRIAL), automatic_result=result,
                    duration_to_stop_s=duration, whole_transfer_completed=False, whole_transfer_duration_s=None,
                    completed_stage_seconds=stages,
                    stopped_stage_elapsed_s=(events[-1]['time_ns']-starts['retract_holding']) / 1e9,
                    terminal_corrections=grasp['terminal_alignment']['corrections'], max_terminal_corrections=5,
                    closure_commands=1, release_commands=0, independent_lift_verified=True,
                    current_robot=robot, terminal_state_audit=str(OUT / 'result.json'),
                    diagnosis=dict(guard='base_moved', source='pregrasp_raised_trial.py:144',
                        translation_limit_m=.015, yaw_limit_rad=.03,
                        post_stop_translation_m=post['current_translation_from_retract_anchor_m'],
                        post_stop_yaw_rad=post['current_yaw_from_retract_anchor_rad'],
                        post_stop_yaw_degrees=math.degrees(post['current_yaw_from_retract_anchor_rad']),
                        exact_offending_pose_logged=False, physical_origin_of_yaw_change_unconfirmed=True),
                    navigation_unchanged=True, production_code_changed=False, model_or_maps_changed=False,
                    robot_commands_sent_during_trial=True, robot_commands_sent_during_audit=False,
                    automatic_retry=False, automatic_resume=False, task_finished_sent=False,
                    next=instruction)
    pointer = 'D:/handyman-training-runs/' + name
    backup = BASE / 'apple-before-r18b-timed-stop-20261022-checkpoints.tar.gz'
    with tarfile.open(backup, 'x:gz') as archive:
        for label, root in roots.items():
            for old in pointers:
                if (root / old).exists():
                    archive.add(root / old, arcname=label + '/' + old)
    for root in roots.values():
        write_json(root / name, progress)
        for old in pointers:
            path = root / old
            if not path.exists():
                continue
            value = read(path)
            value.update(current_state_authoritative_checkpoint=pointer, latest_validation_checkpoint=pointer,
                         latest_live_validation_checkpoint=pointer, latest_full_live_validation_checkpoint=pointer,
                         latest_full_live_status=progress['status'], older_current_fields_are_historical=True,
                         current_robot=robot, next=instruction)
            write_json(path, value)
    digest = sha(BASE / name)
    if any(sha(root / name) != digest for root in roots.values()):
        raise ValueError('checkpoint_sync_mismatch')
    report = dict(status=progress['status'], checkpoint=pointer, sha256=digest, copies=4,
                  duration_to_stop_s=duration, full_transfer_completed=False,
                  terminal_corrections=progress['terminal_corrections'],
                  actual_robot_state=robot['state'], backup=str(backup), robot_commands_sent=False)
    write_json(OUT / 'checkpoint-sync.json', report)
    print(json.dumps(report), flush=True)
