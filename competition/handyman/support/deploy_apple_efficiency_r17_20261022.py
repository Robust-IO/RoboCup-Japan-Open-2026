"""Deploy ONLY verified non-navigation patch, preserving backups and journals."""
import ast
import hashlib
import json
from pathlib import Path
import os
import re
import shutil
import subprocess
import sys
import tarfile
import time

RUNS=Path('/__HANDYMAN_PACKAGE__/runtime')
BASE=json.loads((RUNS/'apple-efficiency-r17-20261022-baseline.json').read_text())
VALIDATION=RUNS/'apple-efficiency-r17-20261022-validation-02'
TESTED=json.loads((VALIDATION/'result.json').read_text())
CANDIDATE=Path(TESTED['candidate'])
RUNTIME=Path(BASE['roots']['runtime'])
REPORT=RUNS/'apple-efficiency-r17-20261022-deployment.json'
CHECKPOINT='apple-efficiency-r17-progress-20261022.json'
POINTERS=['phase5-visual-test-preparation-20261022.json','phase5-restart-checkpoint-20261017.json',
          'apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json',
          'apple-outcome-policy-progress-20261018.json']
ROOTS={'runs':RUNS,**{k:Path(v) for k,v in BASE['roots'].items()}}
READONLY=RUNS/'apple-efficiency-r17-20261022-live-readonly-01/result.json'
def read(path):return json.loads(path.read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
if REPORT.exists() or any((root/CHECKPOINT).exists() for root in ROOTS.values()):
    raise RuntimeError('new_deployment_required')
if TESTED['status']!='candidate_tests_and_recorded_model_reuse_passed':raise RuntimeError('verified_candidate_required')
for rootname in BASE['roots'].values():
    root=Path(rootname)
    for path in [root/'config/AGENTS.md',*(parent/'AGENTS.md' for parent in root.parents)]:
        if path.exists():raise RuntimeError('additional_agent_instructions_require_read:'+str(path))
def navigation_methods(path):
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='LiveBackend')
    return {n.name:ast.dump(n,include_attributes=False) for n in cls.body
            if isinstance(n,ast.FunctionDef) and n.name in ('navigate','route','navigate_source','navigate_destination')}
if navigation_methods(RUNTIME/'apple_transfer_auto_backend.py')!=navigation_methods(CANDIDATE/'apple_transfer_auto_backend.py'):
    raise RuntimeError('navigation_adapter_modified')
active=[]
action_names={'apple_transfer_automation.py','pregrasp_raised_trial.py','reviewed_carry_adjustment.py',
              'run_bedroom_apple_regrasp.py','run_apple_close_lift_trial.py','continue_apple_placement.py'}
for path in Path('/proc').glob('[0-9]*/cmdline'):
    try:argv=[v.decode(errors='replace') for v in path.read_bytes().split(b'\0') if v]
    except (FileNotFoundError,PermissionError,ProcessLookupError):continue
    if '--run' in argv and any(Path(value).name in action_names for value in argv):active.append(dict(pid=path.parent.name,argv=argv))
if active:raise RuntimeError('active_action_prevents_deployment:'+json.dumps(active))
sys.path.insert(0,str(RUNTIME))
from apple_transfer_automation import RobotLock,write_json
with RobotLock():
    for key,rootname in BASE['roots'].items():
        root=Path(rootname)
        for name,expected in BASE['hashes'][key].items():
            if sha(root/name)!=expected:raise RuntimeError('code_changed_since_baseline:'+key+'/'+name)
    for name,expected in TESTED['candidate_hashes'].items():
        if sha(CANDIDATE/name)!=expected:raise RuntimeError('candidate_changed_since_test:'+name)
    metadata={name:read(RUNS/name) for name in POINTERS}
    for name in POINTERS:
        if len({sha(root/name) for root in ROOTS.values()})!=1:raise RuntimeError('metadata_copies_disagree:'+name)
    with tarfile.open(RUNS/'apple-before-efficiency-r17-20261022-checkpoints.tar.gz','x:gz') as archive:
        for label,root in ROOTS.items():
            for name in POINTERS:archive.add(root/name,arcname=label+'/'+name)
    for rootname in BASE['roots'].values():
        root=Path(rootname)
        for name in BASE['files']:
            path=root/name;path.parent.mkdir(parents=True,exist_ok=True)
            temporary=path.with_name(path.name+'.r17-deploy-pending')
            shutil.copyfile(CANDIDATE/name,temporary)
            temporary.replace(path)
    for key,rootname in BASE['roots'].items():
        root=Path(rootname)
        for name,expected in BASE['frozen'][key].items():
            if sha(root/name)!=expected:raise RuntimeError('frozen_file_modified:'+key+'/'+name)
        for name,expected in TESTED['candidate_hashes'].items():
            if sha(root/name)!=expected:raise RuntimeError('deployed_copy_mismatch:'+key+'/'+name)
    environment=Path('/__HANDYMAN_PACKAGE__/project/src/handyman_rebuild_ros2/config/environments/layout_a.yaml')
    if sha(environment)!=BASE['environment_sha256']:raise RuntimeError('environment_changed')
    env=os.environ.copy();env['PYTHONPATH']=str(RUNTIME)+':'+str(RUNTIME/'scripts')
    with (RUNS/'apple-efficiency-r17-20261022-runtime-tests.log').open('x') as log:
        test=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(RUNTIME),'-p','test_*.py'],
            cwd=RUNTIME,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180)
    text=(RUNS/'apple-efficiency-r17-20261022-runtime-tests.log').read_text()
    print(text[-2500:],flush=True)
    if test.returncode:raise RuntimeError('deployed_tests_failed_no_motion_started')
    count=int(re.search(r'Ran (\d+) tests',text).group(1))
    readonly=read(READONLY)
    ready=readonly['checks']['fast-preflight'].get('ready',{})
    previous=read(RUNS/'apple-r16b-full-live-progress-20261022.json')
    next_step=('Read this checkpoint FIRST for CURRENT CODE. efficiency-r17 is deployed/enabled; '
        'all navigation files, waypoints, speed profiles and final tolerances are unchanged. '
        'No new robot motion or whole-transfer r17 timing has been tested. '
        'Previous r16b full run succeeded in542.959s and remains historical motion evidence. '
        'Last observed robot EMPTY/OPEN/TUCKED in living_room after that transfer; apple last confirmed on square_low_table. '
        'Before another full trial, user must restore apple to bedroom white_side_table and fresh-read robot/cameras/localization. '
        'No need to restart Unity just for this update. If Unity was restarted, reinitialize localization from that scene; '
        'never reuse an old numeric anchor/identity or replay old actions. '
        'No automatic action retry, resume, second approach correction or competition Task_finished. '
        'Private shared inference is task-owned and read-only; its warmup overlaps ORIGINAL navigation without changing navigation control.')
    progress=dict(status='non_navigation_efficiency_deployed_pending_new_full_motion_trial',revision='efficiency-r17',
        saved_ns=time.time_ns(),branch=BASE['branch'],wsl_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        enabled=True,configuration=str(RUNTIME/'config/apple_transfer_auto_v1.json'),
        switch='policy.efficient_non_navigation',
        deployment=str(REPORT),candidate_validation=str(VALIDATION/'result.json'),
        code_backup=str(RUNS/'apple-before-efficiency-r17-20261022.tar.gz'),
        deployed_tests=count,real_model_reuse=TESTED['real_model_reuse'],live_readonly_checks=readonly,
        previous_full_live_checkpoint='D:/handyman-training-runs/apple-r16b-full-live-progress-20261022.json',
        previous_full_live_seconds=previous['execution_duration_s'],
        current_robot=dict(previous['current_robot'],base_map_pose=ready.get('base_map_pose'),
                           fresh_state_required_before_next_action=True,last_state_is_not_future_motion_lease=True),
        navigation_files_unchanged=True,navigation_tolerances=[.06,.08,.06],
        speed_limits_changed=False,model_weights_maps_unity_changed=False,
        new_robot_actions_sent=False,new_full_transfer_verified=False,new_full_transfer_duration_s=None,
        original_recordings_unchanged=True,task_finished_sent=False,does_not_exist_sent=False,
        git_commit_or_push_performed=False,next=next_step)
    pointer='D:/handyman-training-runs/'+CHECKPOINT
    updates={CHECKPOINT:progress}
    for name,value in metadata.items():
        value.update(current_state_authoritative_checkpoint=pointer,latest_validation_checkpoint=pointer,
            latest_efficiency_checkpoint=pointer,current_code_revision='efficiency-r17',
            older_current_fields_are_historical=True,next=next_step)
        if name=='apple-transfer-automation-progress-20261017.json':
            value.update(implementation_revision='20261022-efficiency-r17-on-outcome-r16b',
                latest_patch='D:/handyman-training-runs/apple-efficiency-r17-20261022-patch.tar.gz',
                latest_patch_sha256=TESTED['archive_sha256'],
                development_status=progress['status'],latest_code_policy_revision=progress['status'])
        updates[name]=value
    for root in ROOTS.values():
        for name,value in updates.items():write_json(root/name,value)
    hashes={name:sha(RUNS/name) for name in updates}
    if any(sha(root/name)!=digest for root in ROOTS.values() for name,digest in hashes.items()):raise RuntimeError('checkpoint_sync_mismatch')
    report=dict(status='deployed_tests_passed',revision='efficiency-r17',branch=BASE['branch'],copies=3,
        hashes=TESTED['candidate_hashes'],checkpoint_hashes=hashes,checkpoint=pointer,
        archive_sha256=TESTED['archive_sha256'],backup=progress['code_backup'],runtime_tests=count,
        navigation_frozen_hashes_verified=True,navigation_adapter_ast_unchanged=True,navigation_or_speed_parameters_changed=False,
        robot_commands_sent=False,new_full_live_trial=False)
    with REPORT.open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report,indent=2),flush=True)
