"""Fresh terminal evidence and durable checkpoint. No robot action publishers."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import time

BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
TRIAL=BASE/'apple-r19-carry-to-placement-20261022-01'
OUT=BASE/'apple-r19-post-suffix-20261022-01'
sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_plan import map_pose
from task_outcome_policy import robot_ready
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
roots=dict(runs=BASE,runtime=CODE,training=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training'),
           module=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/automation/apple_transfer_v1'))
name='apple-held-carry-r19-progress-20261022.json'
pointers=['phase5-visual-test-preparation-20261022.json','phase5-restart-checkpoint-20261017.json',
          'apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json',
          'apple-outcome-policy-progress-20261018.json','apple-efficiency-r17-progress-20261022.json',
          'apple-terminal-r18-progress-20261022.json']
with RobotLock():
    if any((root/name).exists() for root in roots.values()):raise ValueError('new_checkpoint_required')
    result=read(TRIAL/'result.json')
    if result['status']=='running':raise ValueError('terminal_suffix_required')
    OUT.mkdir(exist_ok=False)
    commands=dict(cameras=[sys.executable,str(CODE/'capture_grasp_scene.py'),str(OUT/'cameras'),
        '--hand-geometry','--map-geometry','--tracking-epoch','r19-post-suffix-20261022-01',
        '--duration','7','--interval','.6','--stop-after-frames','1'],
        readiness=[sys.executable,str(CODE/'inspect_transfer_ready.py'),'--efficient-transfer','--output',str(OUT/'ready.json')])
    children={}
    for key,command in commands.items():
        with (OUT/(key+'.log')).open('x') as stream:
            children[key]=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    codes={key:child.wait(timeout=30) for key,child in children.items()}
    if codes['cameras']:raise ValueError('fresh_terminal_capture_failed')
    capture=sorted((OUT/'cameras').glob('*/snapshot.json'))[-1].parent;snapshot=read(capture/'snapshot.json')
    import rclpy
    rclpy.init();node=rclpy.create_node('r19_suffix_terminal_command_graph_readonly')
    end=time.monotonic()+1
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    topics=['/hsrb/'+tail for tail in ('command_velocity','omni_base_controller/command','arm_trajectory_controller/command',
                                     'gripper_controller/command','head_trajectory_controller/command')]
    pubs={topic:node.count_publishers(topic) for topic in topics}
    node.destroy_node();rclpy.shutdown()
    if any(pubs.values()):raise ValueError('unexpected_motion_owner_after_terminal')
    rows=[json.loads(line) for line in (TRIAL/'journal.jsonl').read_text().splitlines()]
    duration=(rows[-1]['time_ns']-rows[0]['time_ns'])/1e9
    starts={};stages={}
    for row in rows:
        if row['event']=='intent':starts[row['stage']]=row['time_ns']
        elif row['event']=='confirmed':stages[row['stage']]=(row['time_ns']-starts[row['stage']])/1e9
    success=result.get('task_objective_completed') is True
    placement=read(TRIAL/'placement-verification.json') if (TRIAL/'placement-verification.json').exists() else None
    ready=read(OUT/'ready.json')
    if success and not (placement and placement['placement_success_verified']):raise ValueError('actual_placement_proof_required')
    expected={}
    for filename in ('apple-outcome-r16b-20261018-deployment.json','apple-efficiency-r17-20261022-deployment.json',
                     'apple-terminal-r18-20261022-deployment.json','apple-terminal-r18b-20261022-deployment.json',
                     'apple-held-carry-r19-20261022-deployment.json'):
        expected.update(read(BASE/filename)['hashes'])
    if any(sha(CODE/file)!=digest for file,digest in expected.items()):raise ValueError('production_changed_during_suffix')
    pose=map_pose(snapshot)
    robot=dict(state='EMPTY_OPEN_TUCKED_IN_LIVING_ROOM' if success and robot_ready(snapshot['joints']) else 'TERMINAL_STATE_REQUIRES_REVIEW',
        holding_object=False if success else None,arm_is_tucked=robot_ready(snapshot['joints']),
        joints=snapshot['joints'],base_map_pose=pose,room='living_room' if success else None,
        capture=str(capture),depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        command_publishers=pubs,controller_state=ready.get('controller_state'),
        apple_location='Verified on square_low_table#living_room' if success else 'Review current terminal RGBD before any new action',
        fresh_state_required_before_next_action=True,last_state_is_not_future_motion_lease=True)
    instruction=('Read this NEW checkpoint FIRST. held-carry-r19 is deployed; only the verified-held source carry fold '
        'now permits5deg yaw, translation remains15mm; other arm yaw checks remain0.03rad. '
        'Runtime415 tests and recorded r18b-stop regression passed. Source normal fold under r19 has NOT been freshly tested. '
        'An explicitly authorized NEW suffix trial was made after current hand-image/holding/arm/idle checks, '
        'lowering ONLY the already folded arm; it did not replay the failed fold, close or lift. '
        'This suffix is NOT an uninterrupted full-entry transfer and its duration must not be added to203.497s as one full run. '
        + ('Suffix navigation, release and multiframe table placement verification passed. CURRENT EMPTY/OPEN/TUCKED in living_room; '
           'apple is on square_low_table, NOT at bedroom source. Do not repeat source grasp without restoring apple. '
           if success else 'Suffix STOPPED. Read its exact result and fresh current camera before any new action. ')
        + 'No automatic action retry/resume or competition Task_finished. Navigation parameters unchanged. '
        'For the next fair full timed test, restore apple to bedroom white_side_table and start from actual empty/tucked initial scene; '
        'if Unity restarted, freshly revalidate localization, never reuse a numeric anchor or historical identity/action lease. '
        'Old r18b failed journal and original3min23.497s timing remain unchanged; no full-run speedup claim is established.')
    progress=dict(status='r19_reviewed_carry_suffix_and_placement_verified' if success else 'r19_reviewed_carry_suffix_stopped',
        revision='held-carry-r19',saved_ns=time.time_ns(),branch='feat/handyman-phase5-grasping',
        wsl_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        task_id=result['task_id'],trial_directory=str(TRIAL),automatic_result=result,
        suffix_duration_s=duration,is_full_transfer=False,whole_transfer_verified_this_trial=False,
        full_transfer_duration_s=None,completed_stage_seconds=stages,placement_verified=success,
        source_normal_carry_fold_live_retested=False,explicit_current_folded_arm_lowering_live_verified='finish_carry_lowering' in result['confirmed_stages'],
        current_robot=robot,post_run_readiness=ready,
        placement_verification=placement,code_deployment=str(BASE/'apple-held-carry-r19-20261022-deployment.json'),
        runtime_unit_tests=415,local_targeted_unit_tests=50,
        broader_local_checks_note='756-test local source-mirror discovery was not clean: 3 Unity exporter source-string assertions unrelated to these changed files, 4 Unix socket permission errors, 1 skipped. Targeted50 rerun outside sandbox passed; actual runtime candidate415 and deployed415 passed.',
        original_interrupted_full_trial=str(BASE/'automatic-apple-r18b-20261022-01'),
        original_interrupted_duration_s=203.497327455,navigation_parameters_changed=False,
        production_code_changed_during_trial=False,automatic_retry=False,automatic_resume=False,
        old_actions_replayed=False,robot_commands_sent_during_audit=False,
        task_finished_sent=False,git_commit_or_push_performed=False,next=instruction)
    pointer='D:/handyman-training-runs/'+name
    with tarfile.open(BASE/'apple-before-r19-suffix-20261022-checkpoints.tar.gz','x:gz') as archive:
        for label,root in roots.items():
            for old in pointers:
                if (root/old).exists():archive.add(root/old,arcname=label+'/'+old)
    for root in roots.values():
        write_json(root/name,progress)
        for old in pointers:
            if not (root/old).exists():continue
            value=read(root/old);value.update(current_state_authoritative_checkpoint=pointer,
                latest_validation_checkpoint=pointer,latest_live_validation_checkpoint=pointer,
                latest_full_live_validation_checkpoint=pointer,latest_full_live_status=progress['status'],
                current_code_revision='held-carry-r19',older_current_fields_are_historical=True,
                current_robot=robot,next=instruction)
            write_json(root/old,value)
    digest=sha(BASE/name)
    if any(sha(root/name)!=digest for root in roots.values()):raise ValueError('checkpoint_copy_mismatch')
    output=dict(status=progress['status'],suffix_duration_s=duration,placement_verified=success,
        current_robot=robot,stage_seconds=stages,checkpoint=pointer,sha256=digest,copies=4,
        full_transfer_completed_this_trial=False,robot_commands_sent=False,
        placement_summary={key:placement.get(key) for key in ('frames','span_s','center_spread_m','table_edge_safety_verified','robot_ready_for_next_task')} if placement else None)
    write_json(OUT/'result.json',output);print(json.dumps(output),flush=True)
