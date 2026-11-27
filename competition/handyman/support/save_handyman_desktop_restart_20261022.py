"""Save GUI/transport restart readiness, never infer current physical robot state."""
import hashlib
import json
from pathlib import Path
import sys
import tarfile
import time

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
name='phase5-visual-test-preparation-20261022.json';pointer='D:/handyman-training-runs/'+name
roots=dict(runs=base,runtime=code,training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
previous={
    'phase5-restart-checkpoint-20261017.json':'c805fcb1672dc5261161a90a6c373a650b9ca118d1f7c0ddc2368d9a28a96611',
    'apple-transfer-automation-progress-20261017.json':'262564812e2327c0ad6a8926ef69ee18db1b8bb5ecfa49e22a6a061f95479707',
    'apple-transfer-live-progress-20261017.json':'fb3470cc2f21af7fdbfed5288cc5458ada9b211ad6a964a59ee7406474565b44',
    'apple-outcome-policy-progress-20261018.json':'1e47bd8a7dceb69209696e48bc6acf131329aab9194fef888f4c0697a79b152e'}
with RobotLock():
    gui=read(base/'rviz-wsl-desktop-restart-20261022-01.json')
    cameras=read(base/'dual-camera-after-restart-20261022-01.json')
    bridges=read(base/'restart-camera-bridges-20261022-02/startup.json')
    deployment=read(base/'apple-outcome-r16b-20261018-deployment.json')
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if not gui.get('RvizWindowVisible') or gui.get('ShutdownExitCode')!=0 or gui.get('Session')==0:
        raise ValueError('successful_desktop_restart_and_visible_rviz_required')
    if cameras['wsl_boot_id']!=boot or any(cameras['motion_command_publishers'].values()):
        raise ValueError('current_boot_and_no_motion_publishers_required')
    for label,root in roots.items():
        if (root/name).exists():raise ValueError('new_preparation_checkpoint_required')
        for old,digest in previous.items():
            if sha(root/old)!=digest:raise ValueError('progress_changed:'+str(root/old))
        if label!='runs' and any(sha(root/file)!=digest for file,digest in deployment['hashes'].items()):
            raise ValueError('code_changed_since_outcome_r16b')
    robot=dict(state='unknown_after_wsl_restart_pending_fresh_unity',holding_object=None,arm_is_tucked=None,
        base_map_pose=None,layout=None,session=None,apple_location=None,
        fresh_state_required_before_next_action=True,old_r15_pose_and_map_anchor_are_not_current=True)
    next_step=('Read this runtime preparation checkpoint FIRST, then r16b policy. '
        'WSL was explicitly restarted from Windows desktop Session8; dual-camera RViz window was verified visible. '
        'Only transport bridges were restored; no navigation, localization anchor, controller, grasp/place or Moderator task was started. '
        'No Unity image publishers were present during the receive-only check. Ask user to enter/restore Session1/LayoutA and source apple on bedroom white_side_table, '
        'then acquire fresh RGBD/joints/map localization/readiness before any robot action. Do not reuse the 20261018 map anchor, pose, holding state or old trajectories. '
        'For a future cold WSL GUI start, boot RViz from the interactive Windows desktop FIRST, before SSH WSL commands; repeated display-client killing did not fix the Session0 cold-boot issue.')
    result=dict(status='desktop_rviz_visible_waiting_for_unity',saved_ns=time.time_ns(),wsl_boot_id=boot,
        gui_restart=gui,bridges=bridges,camera_check=cameras,current_robot=robot,
        robot_commands_sent=False,task_finished_sent=False,does_not_exist_sent=False,
        navigation_or_localization_started=False,old_map_anchor_restored=False,
        policy_revision='outcome-r16b',code_hashes_still_match_deployment=True,
        model_maps_unity_and_navigation_parameters_changed=False,
        latest_policy_checkpoint='D:/handyman-training-runs/apple-outcome-policy-progress-20261018.json',
        next=next_step)
    updates={name:result}
    for old in previous:
        value=read(base/old)
        value.update(latest_runtime_preparation_checkpoint=pointer,current_state_authoritative_checkpoint=pointer,
            current_runtime_state=result['status'],old_physical_state_requires_new_observation=True,next=next_step)
        if old!='apple-outcome-policy-progress-20261018.json':
            value.update(current_robot=robot,latest_robot=robot)
        updates[old]=value
    backup=base/'apple-before-desktop-restart-20261022-checkpoints.tar.gz'
    with tarfile.open(backup,'x:gz') as archive:
        for label,root in roots.items():
            for old in previous:archive.add(root/old,arcname=label+'/'+old)
    for root in roots.values():
        for filename,value in updates.items():write_json(root/filename,value)
    hashes={filename:sha(base/filename) for filename in updates}
    if any(sha(root/filename)!=digest for root in roots.values() for filename,digest in hashes.items()):
        raise ValueError('metadata_copy_mismatch')
    report=dict(status='desktop_restart_preparation_synced',copies=4,hashes=hashes,backup=str(backup),
        robot_commands_sent=False,new_robot_state_verified=False)
    with (base/'phase5-visual-test-preparation-20261022-sync.json').open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report),flush=True)
