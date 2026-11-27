"""Backed-up code-only r19 deployment. No ROS action entry is invoked."""
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile

RUNS=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
REPO=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
ROOTS=dict(runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),
           training=REPO/'training',module=REPO/'training/automation/apple_transfer_v1')
CODE=ROOTS['runtime']
PREFIX='apple-held-carry-r19-20261022'
PATCH=RUNS/(PREFIX+'-patch.tar.gz')
EXPECTED='4e75f9ef7692ccfee13ac638c807ee06bae79a5b35b5a1117e4e90b45dbea540'
FILES=['pregrasp_raised_trial.py','task_outcome_policy.py','test_task_outcome_policy.py','APPLE_TRANSFER_AUTOMATION.txt']
FROZEN=['navigate_carry_trial.py','motion_speed.py','config/motion_speed_profiles.json',
        'config/apple_transfer_auto_v1.json','apple_transfer_auto_backend.py','apple_transfer_automation.py',
        'run_apple_close_lift_trial.py','run_bedroom_apple_regrasp.py','grasp_terminal_correction.py',
        'reviewed_carry_adjustment.py','table_release_gate.py','verify_table_placement.py',
        'capture_grasp_scene.py','apple_shared_inference.py','apple_inference_worker.py']
VALIDATION=RUNS/(PREFIX+'-validation-01')
REPORT=RUNS/(PREFIX+'-deployment.json')
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
def write_new(path,value):
    with path.open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False)
def tests(root,log):
    env=dict(os.environ,PYTHONPATH=str(root)+':'+str(root/'scripts'))
    with log.open('x') as stream:
        proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(root),'-p','test_*.py'],
                            cwd=root,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=180)
    output=log.read_text();print(output[-2000:],flush=True)
    if proc.returncode:raise RuntimeError('offline_tests_failed:'+str(log))
    return int(re.search(r'Ran (\d+) tests',output).group(1))
if REPORT.exists() or VALIDATION.exists():raise ValueError('new_deployment_required')
for root in ROOTS.values():
    for path in [root/'AGENTS.md',root/'config/AGENTS.md',*(p/'AGENTS.md' for p in root.parents)]:
        if path.exists():
            print(str(path)+'\n'+path.read_text(),flush=True)
            raise ValueError('read_applicable_instructions_first')
if sha(PATCH)!=EXPECTED:raise ValueError('patch_hash_mismatch')
branch=subprocess.check_output(['git','-C',str(REPO),'branch','--show-current'],text=True).strip()
if branch!='feat/handyman-phase5-grasping':raise ValueError('unexpected_branch')
expected={}
for name in ('apple-outcome-r16b-20261018-deployment.json','apple-efficiency-r17-20261022-deployment.json',
             'apple-terminal-r18-20261022-deployment.json','apple-terminal-r18b-20261022-deployment.json'):
    expected.update(read(RUNS/name)['hashes'])
for name,digest in expected.items():
    if sha(CODE/name)!=digest:raise ValueError('unexpected_runtime_change:'+name)
baseline={label:{name:sha(root/name) for name in FILES+FROZEN} for label,root in ROOTS.items()}
for name in FILES:
    if len({baseline[k][name] for k in ROOTS})!=1:raise ValueError('source_copies_disagree:'+name)
environment=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/src/handyman_rebuild_ros2/config/environments/layout_a.yaml')
environment_hash=sha(environment)
VALIDATION.mkdir()
write_new(RUNS/(PREFIX+'-baseline.json'),dict(hashes=baseline,branch=branch,environment_sha256=environment_hash,robot_commands_sent=False))
candidate=VALIDATION/'candidate'
shutil.copytree(CODE,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git'))
with tarfile.open(PATCH) as archive:
    if set(archive.getnames())!=set(FILES):raise ValueError('unexpected_patch_members')
    for member in archive.getmembers():
        if not member.isfile() or Path(member.name).is_absolute() or '..' in Path(member.name).parts:
            raise ValueError('unsafe_patch_member')
        with (candidate/member.name).open('wb') as stream:stream.write(archive.extractfile(member).read())
for name in FILES:
    if name.endswith('.py'):ast.parse((candidate/name).read_text(),filename=name)
for name in FROZEN:
    if sha(candidate/name)!=baseline['runtime'][name]:raise ValueError('frozen_file_modified:'+name)
candidate_tests=tests(candidate,VALIDATION/'candidate-tests.log')
sys.path.insert(0,str(candidate))
from task_outcome_policy import arm_base_stability
old=read(RUNS/'apple-r18b-post-stopped-20261022-01/result.json')
before=old['retract_anchor'];after=old['current_base_map_pose']
if arm_base_stability(before,after)['allowed'] or not arm_base_stability(before,after,carrying=True)['allowed']:
    raise ValueError('recorded_stop_regression_not_reproduced')
replay=dict(original_guard_reproduced=True,held_carry_now_accepts_recorded_small_yaw=True,
            result=arm_base_stability(before,after,carrying=True),robot_commands_sent=False,
            historical_failed_trial_not_modified=True)
write_new(VALIDATION/'recorded-replay.json',replay)
actions={'apple_transfer_automation.py','pregrasp_raised_trial.py','reviewed_carry_adjustment.py',
         'run_bedroom_apple_regrasp.py','run_apple_close_lift_trial.py','continue_apple_placement.py'}
for path in Path('/proc').glob('[0-9]*/cmdline'):
    try:argv=[v.decode(errors='replace') for v in path.read_bytes().split(b'\0') if v]
    except (FileNotFoundError,PermissionError,ProcessLookupError):continue
    if '--run' in argv and any(Path(v).name in actions for v in argv):raise ValueError('active_motion_prevents_deploy')
from apple_transfer_automation import RobotLock
with RobotLock():
    for label,root in ROOTS.items():
        for name,digest in baseline[label].items():
            if sha(root/name)!=digest:raise ValueError('concurrent_change:'+str(root/name))
    with tarfile.open(RUNS/'apple-before-held-carry-r19-20261022.tar.gz','x:gz') as archive:
        for label,root in ROOTS.items():
            for name in FILES:archive.add(root/name,arcname=label+'/'+name)
    hashes={name:sha(candidate/name) for name in FILES}
    for root in ROOTS.values():
        for name in FILES:
            dest=root/name;temp=dest.with_name(dest.name+'.r19-pending')
            shutil.copyfile(candidate/name,temp);temp.replace(dest)
    for label,root in ROOTS.items():
        for name in FROZEN:
            if sha(root/name)!=baseline[label][name]:raise ValueError('frozen_file_changed:'+name)
        for name,digest in hashes.items():
            if sha(root/name)!=digest:raise ValueError('deployed_copy_mismatch:'+name)
    runtime_tests=tests(CODE,VALIDATION/'runtime-tests.log')
    if sha(environment)!=environment_hash:raise ValueError('environment_changed')
    result=dict(status='deployed_tests_and_recorded_regression_passed',revision='held-carry-r19',
                branch=branch,files=FILES,hashes=hashes,copies=3,candidate_tests=candidate_tests,
                runtime_tests=runtime_tests,recorded_replay=replay,validation=str(VALIDATION),
                archive_sha256=EXPECTED,backup=str(RUNS/'apple-before-held-carry-r19-20261022.tar.gz'),
                navigation_files_and_parameters_unchanged=True,robot_commands_sent=False,
                new_live_validation=False,git_commit_or_push_performed=False)
    write_new(REPORT,result)
    print(json.dumps(result),flush=True)
