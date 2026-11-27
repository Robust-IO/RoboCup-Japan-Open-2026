"""Fresh receive-only terminal capture and checkpoint; NEVER recovery motion."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import time

BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
ROOT=BASE/'automatic-apple-r17-20261022-01'
OUT=BASE/'apple-r17-post-stopped-20261022-01'
sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_plan import map_pose
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
name='apple-r17-full-live-progress-20261022.json'
roots=dict(runs=BASE,runtime=CODE,training=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training'),
    module=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/automation/apple_transfer_v1'))
pointers=['phase5-visual-test-preparation-20261022.json','phase5-restart-checkpoint-20261017.json',
          'apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json',
          'apple-outcome-policy-progress-20261018.json','apple-efficiency-r17-progress-20261022.json']
with RobotLock():
    result=read(ROOT/'result.json')
    if result['status']!='stopped' or result['stage']!='grasp_and_lift':raise RuntimeError('specific_terminal_trial_required')
    if any((root/name).exists() for root in roots.values()):raise RuntimeError('checkpoint_exists')
    OUT.mkdir(exist_ok=False)
    capture=OUT/'cameras'
    with (OUT/'capture.log').open('x') as log:
        child=subprocess.run([sys.executable,str(CODE/'capture_grasp_scene.py'),str(capture),
            '--hand-geometry','--map-geometry','--tracking-epoch','r17-post-stopped-20261022-01',
            '--duration','7','--interval','.6','--stop-after-frames','1'],stdout=log,stderr=subprocess.STDOUT,timeout=25)
    if child.returncode:raise RuntimeError('terminal_capture_failed')
    last=sorted(capture.glob('*/snapshot.json'))[-1].parent;snapshot=read(last/'snapshot.json')
    import rclpy
    rclpy.init();node=rclpy.create_node('r17_terminal_command_graph_readonly')
    deadline=time.monotonic()+1.
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    topics=['/hsrb/'+tail for tail in ('command_velocity','omni_base_controller/command',
        'arm_trajectory_controller/command','gripper_controller/command','head_trajectory_controller/command')]
    publishers={topic:node.count_publishers(topic) for topic in topics}
    node.destroy_node();rclpy.shutdown()
    if any(publishers.values()):raise RuntimeError('unexpected_motion_owner_after_stop')
    events=[json.loads(line) for line in (ROOT/'journal.jsonl').read_text().splitlines()]
    duration=(events[-1]['time_ns']-events[0]['time_ns'])/1e9
    starts={};stages={}
    for event in events:
        if event['event']=='intent':starts[event['stage']]=event['time_ns']
        elif event['event']=='confirmed':stages[event['stage']]=(event['time_ns']-starts[event['stage']])/1e9
    diagnosis=read(ROOT/'stopped-readonly-diagnosis.json')
    robot=dict(state='EMPTY_OPEN_EXTENDED_AT_BEDROOM_SOURCE',holding_object=False,arm_is_tucked=False,
        hand_motor=snapshot['joints']['hand_motor_joint'],joints=snapshot['joints'],
        base_map_pose=map_pose(snapshot),room='bedroom',capture=str(last),
        depth_stamp_ns=snapshot['streams']['depth']['stamp_ns'],command_publishers=publishers,
        apple_location='Still ungrasped at source table; no closure/lift/release occurred.',
        fresh_state_required_before_next_action=True,last_state_is_not_future_motion_lease=True)
    instruction=('Read this NEW terminal checkpoint FIRST. r17 timed full trial01 STOPPED before closure at168.419s. '
        'One49.36cm approach finished with0.90mm odom error, but fresh visual target was26.4mm forward and19.6mm lateral '
        'from contact center (old gates20/12mm). No second movement, closure, lift, carry, release or placement. '
        'CURRENT last-confirmed EMPTY/OPEN/EXTENDED in BEDROOM at source, NOT empty/tucked in living_room. '
        'All command publishers zero; RViz left open. No automatic resume/action replay or further motion. '
        'Diagnose far-to-near measurement/one-shot alignment before another authorized trial. '
        'Do not simply loosen gates or assume measurement drift proves an object collision. '
        'Old r16b success remains historical; this failed prefix is NOT an optimized complete-transfer time. '
        'Navigation and all code remained unchanged during timed test.')
    progress=dict(status='r17_full_timed_trial_stopped_before_closure',revision='efficiency-r17',saved_ns=time.time_ns(),
        branch='feat/handyman-phase5-grasping',wsl_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        task_id=result['task_id'],trial_directory=str(ROOT),automatic_result=result,
        duration_to_stop_s=duration,whole_transfer_completed=False,whole_transfer_duration_s=None,
        stage_seconds=stages,current_robot=robot,diagnosis=str(ROOT/'stopped-readonly-diagnosis.json'),
        last_recorded_visual_alignment=diagnosis['rows'][-1],
        approach_commands=1,closure_commands=0,lift_commands=0,release_commands=0,
        robot_actions_sent_during_trial=True,robot_actions_sent_during_audit=False,
        automatic_retry=False,automatic_resume=False,task_finished_sent=False,does_not_exist_sent=False,
        code_navigation_model_maps_unity_changed_during_trial=False,
        previous_success='D:/handyman-training-runs/apple-r16b-full-live-progress-20261022.json',next=instruction)
    updates={name:progress};pointer='D:/handyman-training-runs/'+name
    with tarfile.open(BASE/'apple-before-r17-full-live-stop-20261022-checkpoints.tar.gz','x:gz') as archive:
        for label,root in roots.items():
            for old in pointers:archive.add(root/old,arcname=label+'/'+old)
    for old in pointers:
        value=read(BASE/old)
        value.update(current_state_authoritative_checkpoint=pointer,latest_validation_checkpoint=pointer,
            latest_live_validation_checkpoint=pointer,latest_full_live_validation_checkpoint=pointer,
            latest_full_live_status=progress['status'],older_current_fields_are_historical=True,
            current_robot=robot,next=instruction)
        if old=='apple-efficiency-r17-progress-20261022.json':
            value.update(new_full_transfer_verified=False,new_full_transfer_duration_s=None,
                         latest_timed_trial_duration_to_stop_s=duration,latest_timed_trial_status=progress['status'])
        updates[old]=value
    for root in roots.values():
        for filename,value in updates.items():write_json(root/filename,value)
    hashes={filename:sha(BASE/filename) for filename in updates}
    if any(sha(root/filename)!=digest for root in roots.values() for filename,digest in hashes.items()):raise RuntimeError('checkpoint_copy_mismatch')
    report=dict(status=progress['status'],duration_to_stop_s=duration,current_robot=robot,checkpoint=pointer,
        checkpoint_hashes=hashes,stage_seconds=stages,robot_commands_sent=False)
    with (OUT/'result.json').open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report,indent=2),flush=True)
