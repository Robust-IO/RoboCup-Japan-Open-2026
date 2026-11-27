"""Continue ONLY the currently RGBD-audited held can: carry then place.

Original failed full-run verdict remains intact. This is explicitly a reviewed
continuation, never an uninterrupted full-start success. No grasp/close/lift.
"""
import hashlib
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_backend import stop_group
from can_transfer_context import build_context

base=Path('/__HANDYMAN_PACKAGE__/runtime');original=base/'can-auto-live-20261023-01'
audit_root=base/'can-cap-axis-v1-20261023-current-verification';root=base/'can-reviewed-carry-place-20261023-01'
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old=read(original/'grasp/result.json');audit=read(audit_root/'result.json');old_meta=read(original/'run-meta.json')
deployment=read(base/'can-cap-axis-v1-20261023-deployment.json')
if audit['status']!='current_can_held_after_tilted_grasp_verified' or len(audit['rows'])!=3:raise ValueError('actual_current_held_audit_required')
if old_meta['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():raise ValueError('same_boot_required')
if any(sha(code/n)!=digest for n,digest in deployment['hashes'].items()):raise ValueError('reviewed_cap_axis_code_required')
failed_hashes={str(original/'result.json'):sha(original/'result.json'),str(original/'grasp/result.json'):sha(original/'grasp/result.json')}
root.mkdir(exist_ok=False);active=None;report=dict(status='starting',run_kind='reviewed_held_continuation_after_tilt_geometry_fix',
    original_failed_full_run=str(original),grasp_or_close_or_lift_replayed=False,
    physical_transfer_completed=False,uninterrupted_full_transfer_verified=False,task_finished_sent=False,
    started_ns=time.time_ns(),confirmed_stages=[])
def save():write_json(root/'result.json',report)
def interrupted(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,interrupted)
with RobotLock() as lock,(root/'journal.jsonl').open('x') as stream:
    def record(event):
        value=dict(time_ns=time.time_ns(),**event);stream.write(json.dumps(value,allow_nan=False)+'\n');stream.flush();print(json.dumps(value),flush=True)
    try:
        write_json(root/'run-meta.json',dict(version=1,run_id=root.name,mode='live',object='canned_juice',
            boot_id=old_meta['boot_id'],started_ns=old_meta['started_ns'],continuation_created_ns=time.time_ns(),
            run_kind=report['run_kind'],original_run_id=old_meta['run_id'],fresh_holding_audit=str(audit_root)))
        before=root/'grasp/captures/preclose';before.mkdir(parents=True)
        def copy_capture(source,destination):
            for name in ('rgb.png','depth_m.npy','snapshot.json','can-measurement.json'):
                shutil.copyfile(source/name,destination/name)
                if sha(source/name)!=sha(destination/name):raise ValueError('raw_evidence_copy_mismatch')
        copy_capture(original/'grasp/captures/preclose',before);rows=[]
        for i,row in enumerate(audit['rows']):
            target=root/f'grasp/captures/audited-held-{i:02d}';target.mkdir();copy_capture(Path(row['capture']),target)
            rows.append(dict(row['proof'],capture=str(target)))
        canonical=dict(status='can_grasp_and_lift_verified',grasp_success_verified=True,task_id='grasp',
            verification_mode='independent_current_rgbd_review_after_original_fit_failure',
            original_failed_result=str(original/'grasp/result.json'),source_audit=str(audit_root/'result.json'),
            close=dict(old['close'],before_capture=str(before)),lift=old['lift'],grasp_plan=old['grasp_plan'],
            reference=old['reference'],held_opening_rad=audit['holding_opening_rad'],lift_verification=rows,
            actual_grasp_or_close_or_lift_replayed=False,uninterrupted_full_transfer_verified=False)
        write_json(root/'grasp/result.json',canonical);context=build_context(root)
        for stage,script,extra,expected in [
            ('carry','run_canned_carry_trial.py',['--navigate-destination'],'can_retraction_and_destination_navigation_verified'),
            ('place','run_canned_place_trial.py',[],'reviewed_can_placement_verified')]:
            report['stage']=stage;save()
            command=[sys.executable,str(code/script),'--run','--context',str(context),'--output',str(root/stage),
                '--owner-lock-fd',str(lock.stream.fileno()),'--owner-run-root',str(root),*extra]
            record(dict(event='command_intent',stage=stage,command=command))
            with (root/(stage+'.log')).open('x') as log:
                active=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,pass_fds=(lock.stream.fileno(),))
                code_result=active.wait();active=None
            outcome=read(root/stage/'result.json')
            if code_result or outcome['status']!=expected:raise ValueError('continuation_stage_failed:'+stage+':'+str(outcome.get('reason')))
            report['confirmed_stages'].append(stage);save();record(dict(event='command_confirmed',stage=stage))
        placed=read(root/'place/result.json');verification=read(root/'place/placement-verification.json')
        if not placed['placement_success_verified'] or not verification['placement_success_verified']:raise ValueError('actual_final_placement_required')
        report.update(status='reviewed_can_carry_and_placement_verified',physical_transfer_completed=True,
            robot_ready_for_next_task=placed['robot_ready_for_next_task'],last_fresh_robot_state=placed['last_fresh_robot_state'],
            final_rgb_capture=placed['final_placement_rows'][-1]['capture'])
        if any(sha(Path(p))!=digest for p,digest in failed_hashes.items()):raise ValueError('original_failed_result_changed')
        report.update(original_failed_verdict_preserved=True,tests=deployment['tests'])
    except (Exception,KeyboardInterrupt) as exc:
        if active:stop_group(active);active=None
        report.update(status='stopped_requires_actual_state_review',reason=str(exc) or 'interrupted',automatic_resume=False)
    finally:
        report['completed_ns']=time.time_ns();report['elapsed_s']=(report['completed_ns']-report['started_ns'])/1e9;save();print(json.dumps(report),flush=True)
if report['status']!='reviewed_can_carry_and_placement_verified':raise SystemExit(1)
name='canned-reviewed-continuation-progress-20261023.json'
repo=Path('/__HANDYMAN_PACKAGE__/project');roots=[base,code,repo/'training',repo/'training/automation/apple_transfer_v1']
checkpoint=dict(report,status='can_physical_transfer_completed_after_tilt_fix_review',
    actual_continuation='D:/handyman-training-runs/'+root.name,
    boot_id=old_meta['boot_id'],latest_deployment='D:/handyman-training-runs/can-cap-axis-v1-20261023-deployment.json',
    source='white_side_table#bedroom_1',destination='square_low_table#living_room',
    final_image=report['final_rgb_capture'].replace('/mnt/d/','D:/'),
    next=['Can is now RELEASED on living_room square_low_table; do not rerun a held continuation.',
        'The first unified trial stopped after one successful physical grasp/lift because an upright-only geometry fit rejected a13.7degree tilt.',
        'The correction tracks actual cap normal and actual source-colored side depth, preserving radius limits and fitting tilted bottom/support dimensions.',
        'New current RGBD verified the held object before carry/place continuation. Original failed verdicts are preserved; uninterrupted_full_transfer_verified remains FALSE.',
        'Read latest cap-axis deployment hashes, not the earlier continuous-entry manifest. A fresh-start whole-run regression is still required.'])
if any((p/name).exists() for p in roots):raise ValueError('new_checkpoint_required')
data=json.dumps(checkpoint,indent=2,allow_nan=False).encode()
for p in roots:
    with (p/name).open('xb') as f:f.write(data)
print(json.dumps(dict(checkpoint='D:/handyman-training-runs/'+name,sha256=hashlib.sha256(data).hexdigest(),
    physical_transfer_completed=True,uninterrupted_full_transfer_verified=False)),flush=True)
