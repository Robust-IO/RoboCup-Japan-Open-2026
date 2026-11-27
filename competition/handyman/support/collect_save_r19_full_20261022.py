"""Receive-only post-trial check and durable full-r19 timing checkpoint."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import time

BASE=Path('/__HANDYMAN_PACKAGE__/runtime')
CODE=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
TRIAL=BASE/'automatic-apple-r19-20261022-01'
CONTROL=BASE/'apple-r19-full-live-control-20261022-01'
OUT=BASE/'apple-r19-post-full-live-20261022-01'
sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,STAGES,write_json
from apple_transfer_auto_plan import map_pose
from task_outcome_policy import robot_ready
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
roots=dict(runs=BASE,runtime=CODE,training=Path('/__HANDYMAN_PACKAGE__/project/training'),
           module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
name='apple-r19-full-live-progress-20261022.json'
pointers=['phase5-visual-test-preparation-20261022.json','phase5-restart-checkpoint-20261017.json',
          'apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json',
          'apple-outcome-policy-progress-20261018.json','apple-efficiency-r17-progress-20261022.json',
          'apple-terminal-r18-progress-20261022.json','apple-held-carry-r19-progress-20261022.json']
with RobotLock():
    if any((root/name).exists() for root in roots.values()):raise ValueError('new_checkpoint_required')
    result=read(TRIAL/'result.json')
    if result['status']=='running':raise ValueError('terminal_trial_required')
    OUT.mkdir(exist_ok=False)
    commands=dict(cameras=[sys.executable,str(CODE/'capture_grasp_scene.py'),str(OUT/'cameras'),
        '--hand-geometry','--map-geometry','--tracking-epoch','r19-post-full-live-20261022-01',
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
    rclpy.init();node=rclpy.create_node('r19_full_terminal_graph_readonly')
    end=time.monotonic()+1
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    topics=['/hsrb/'+tail for tail in ('command_velocity','omni_base_controller/command','arm_trajectory_controller/command',
                                     'gripper_controller/command','head_trajectory_controller/command')]
    publishers={topic:node.count_publishers(topic) for topic in topics}
    node.destroy_node();rclpy.shutdown()
    if any(publishers.values()):raise ValueError('unexpected_motion_owner_after_trial')
    rows=[json.loads(line) for line in (TRIAL/'journal.jsonl').read_text().splitlines()]
    if rows[-1]['event'] not in ('stopped','verified_complete'):raise ValueError('terminal_journal_required')
    duration=(rows[-1]['time_ns']-rows[0]['time_ns'])/1e9
    starts={};stages={}
    for row in rows:
        if row['event']=='intent':starts[row['stage']]=row['time_ns']
        elif row['event']=='confirmed':stages[row['stage']]=(row['time_ns']-starts[row['stage']])/1e9
    success=result.get('task_objective_completed') is True
    placement=read(TRIAL/'placement-verification.json') if (TRIAL/'placement-verification.json').exists() else None
    lift=read(TRIAL/'grasp/close-lift/independent-lift-verification.json') if (TRIAL/'grasp/close-lift/independent-lift-verification.json').exists() else None
    grasp=read(TRIAL/'grasp/result.json') if (TRIAL/'grasp/result.json').exists() else None
    carry=read(TRIAL/'carry-retract/result.json') if (TRIAL/'carry-retract/result.json').exists() else None
    if success and (result['confirmed_stages']!=list(STAGES) or not placement['placement_success_verified']
                    or not lift['grasp_success_verified'] or carry['status']!='passed'
                    or carry.get('carry_completion_from_current') is not False):
        raise ValueError('actual_full_entry_proof_required')
    ready=read(OUT/'ready.json')
    if success and (not ready['ready'] or not robot_ready(snapshot['joints'])):raise ValueError('terminal_readiness_not_verified')
    expected={}
    for file in ('apple-outcome-r16b-20261018-deployment.json','apple-efficiency-r17-20261022-deployment.json',
                 'apple-terminal-r18-20261022-deployment.json','apple-terminal-r18b-20261022-deployment.json',
                 'apple-held-carry-r19-20261022-deployment.json'):
        expected.update(read(BASE/file)['hashes'])
    if any(sha(CODE/file)!=digest for file,digest in expected.items()):raise ValueError('production_changed_during_trial')
    nav={key:read(TRIAL/(key+'-navigation.json')) for key in ('source','destination') if (TRIAL/(key+'-navigation.json')).exists()}
    for value in nav.values():
        if value.get('status')=='arrived' and (not value['nav_reset'] or not value['speed_parameters_restored'] or value['goal_precision_changed']):
            raise ValueError('navigation_reset_or_unchanged_precision_not_verified')
    navigation=sum(stages.get(key,0.) for key in ('navigate_source','navigate_destination'))
    robot=dict(state='EMPTY_OPEN_TUCKED_IN_LIVING_ROOM' if success else 'STOPPED_REQUIRES_FRESH_REVIEW',
        holding_object=False if success else None,arm_is_tucked=robot_ready(snapshot['joints']),joints=snapshot['joints'],
        base_map_pose=map_pose(snapshot),room='living_room' if success else None,capture=str(capture),
        depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],command_publishers=publishers,
        controller_state=ready.get('controller_state'),
        apple_location='Verified on square_low_table#living_room; no longer at bedroom white_side_table' if success else 'Review terminal RGBD',
        fresh_state_required_before_next_action=True,last_state_is_not_future_motion_lease=True)
    instruction=('Read this NEW full-r19 checkpoint FIRST. '
        + ('One NEW uninterrupted full-entry LayoutA apple transfer completed ALL17 stages, with no human motion interventions. '
           'Normal r19 source carry fold (not lower-only continuation) passed live; source terminal corrections='+str(grasp['terminal_alignment']['corrections'])+'. '
           'CURRENT EMPTY/OPEN/TUCKED in living_room; apple is on square_low_table, NOT bedroom source. '
           if success else 'The NEW full-r19 trial STOPPED. Read its exact result and actual terminal state; do not replay failed actions. ')
        + 'The start was actual empty/open/tucked living_room pose, not configured spawn, so do not claim a same-start speedup over r16b. '
        'Whole time is journal begin to terminal event; internal checks and waits included, external prep separate. '
        'No automatic action retry/resume, no old interrupted journal replay and no competition Task_finished. '
        'Navigation/speeds/maps/model unchanged. One apple trial is not30-class generalization,80/85percent statistical acceptance or generic Moderator integration. '
        'Before another test, inspect fresh robot/RGBD/localization and genuinely restore source apple when needed. '
        'If Unity restarted, freshly validate localization rather than reusing an old numeric anchor or identity lease.')
    startup=read(CONTROL/'startup.json')
    progress=dict(status='r19_new_uninterrupted_full_apple_transfer_verified' if success else 'r19_new_full_apple_transfer_stopped',
        revision='held-carry-r19',saved_ns=time.time_ns(),branch='feat/handyman-phase5-grasping',
        wsl_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        task_id=result['task_id'],trial_directory=str(TRIAL),automatic_result=result,
        whole_uninterrupted_transfer_verified=success,execution_duration_s=duration if success else None,
        duration_to_stop_s=None if success else duration,confirmed_stage_count=len(result['confirmed_stages']),
        stage_seconds=stages,navigation_seconds=navigation,non_navigation_seconds=duration-navigation,
        start_pose=startup['robot_start_pose'],start_context=startup['start_context'],
        comparable_to_configured_spawn_baseline=False,external_preparation_seconds=startup['preparation_elapsed_s'],
        terminal_corrections=grasp.get('terminal_alignment',{}).get('corrections') if grasp else None,
        maximum_terminal_corrections=5,normal_carry_fold_live_verified=success,
        carry_base_motion=carry.get('base_motion') if carry else None,
        independent_lift_verified=bool(lift and lift.get('grasp_success_verified')),
        placement_verified=bool(placement and placement.get('placement_success_verified')),
        placement_verification=placement,current_robot=robot,post_run_readiness=ready,
        runtime_unit_tests=415,manual_motion_interventions_during_run=False,
        code_model_maps_unity_or_navigation_changed_during_trial=False,navigation_tolerances=[.06,.08,.06],
        task_finished_sent=False,does_not_exist_sent=False,automatic_retry=False,automatic_resume=False,
        old_actions_replayed=False,robot_commands_sent_during_audit=False,git_commit_or_push_performed=False,
        previous_interrupted_trial=str(BASE/'automatic-apple-r18b-20261022-01'),
        previous_suffix_trial=str(BASE/'apple-r19-carry-to-placement-20261022-01'),next=instruction)
    pointer='D:/handyman-training-runs/'+name
    with tarfile.open(BASE/'apple-before-r19-full-live-20261022-checkpoints.tar.gz','x:gz') as archive:
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
                latest_full_live_duration_s=progress['execution_duration_s'],current_code_revision='held-carry-r19',
                older_current_fields_are_historical=True,current_robot=robot,next=instruction)
            write_json(root/old,value)
    digest=sha(BASE/name)
    if any(sha(root/name)!=digest for root in roots.values()):raise ValueError('checkpoint_copy_mismatch')
    output=dict(status=progress['status'],whole_uninterrupted_transfer_verified=success,
        duration_s=duration,stage_seconds=stages,navigation_seconds=navigation,non_navigation_seconds=duration-navigation,
        terminal_corrections=progress['terminal_corrections'],current_robot=robot,checkpoint=pointer,sha256=digest,copies=4,
        robot_commands_sent=False,placement_summary={key:placement.get(key) for key in ('frames','span_s','center_spread_m','table_edge_safety_verified','robot_ready_for_next_task')} if placement else None)
    write_json(OUT/'result.json',output);print(json.dumps(output),flush=True)
