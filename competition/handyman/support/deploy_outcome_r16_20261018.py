"""Isolated tests + recorded RGBD audit, then backed-up code deployment; no motion."""
import fcntl
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile

base=Path('/__HANDYMAN_PACKAGE__/runtime')
archive=base/'apple-outcome-r16-20261018-patch.tar.gz'
candidate=base/'apple-outcome-r16-candidate-20261018-01'
backup=base/'apple-before-outcome-r16-20261018.tar.gz'
report_path=base/'apple-outcome-r16-20261018-deployment.json'
names=set('task_outcome_policy.py test_task_outcome_policy.py apple_transfer_auto_plan.py table_release_gate.py verify_apple_lift_sequence.py audit_preclose_identity_lift_follow.py verify_table_placement.py apple_transfer_auto_backend.py apple_transfer_automation.py inverse_table_place_plan.py pregrasp_raised_trial.py continue_apple_placement.py reviewed_carry_adjustment.py test_table_release_gate.py test_verify_apple_lift_sequence.py test_apple_transfer_auto_backend.py test_parent_bound_lift_follow.py test_reviewed_carry_adjustment.py APPLE_TRANSFER_AUTOMATION.txt'.split())
repo=Path('/__HANDYMAN_PACKAGE__/project')
roots=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),training=repo/'training',module=repo/'training/automation/apple_transfer_v1')
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()

def tests(root,label):
    log=base/f'apple-outcome-r16-20261018-{label}-tests.log'
    with log.open('x') as stream:
        child=subprocess.run([sys.executable,'-m','unittest','discover','-p','test_*.py','-v'],cwd=root,stdout=stream,stderr=subprocess.STDOUT)
    value=log.read_text();found=re.findall(r'Ran (\d+) tests?',value)
    print('\n'.join(value.splitlines()[-5:]),flush=True)
    return dict(exit=child.returncode,count=int(found[-1]) if found else None,log=str(log))

lock=open('/tmp/handyman-apple-transfer-automation.lock','a+')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
branch=subprocess.check_output(['git','branch','--show-current'],cwd=repo,text=True).strip()
if branch!='feat/handyman-phase5-grasping':raise ValueError('unexpected_branch')
if sha(archive)!='f6d3fb2dd8afe72739555dee301f9ad38847ea9d813b68ba3a7626b6bf24ded2':raise ValueError('exact_patch_required')
if report_path.exists():raise ValueError('new_deployment_report_required')
before={key:{name:sha(root/name) if (root/name).exists() else None for name in names} for key,root in roots.items()}
for name in names:
    if len({row[name] for row in before.values() if row[name] is not None})>1:raise ValueError('preexisting_copy_difference:'+name)
    # These four entry/test files originally live only in runtime/module; the
    # training-root copy is new, not an overwritten conflicting user edit.
    existing={key for key,row in before.items() if row[name] is not None}
    allowed_missing_training={'apple_transfer_automation.py','inverse_table_place_plan.py','continue_apple_placement.py','test_verify_apple_lift_sequence.py'}
    if existing and existing!=set(roots) and not (existing=={'runtime','module'} and name in allowed_missing_training):
        raise ValueError('unexpected_missing_copy:'+name)
shutil.copytree(roots['runtime'],candidate,ignore=shutil.ignore_patterns('__pycache__'))
with tarfile.open(archive,'r:gz') as patch:
    members=patch.getmembers()
    if len(members)!=len(names) or {m.name for m in members}!=names or any(not m.isfile() for m in members):raise ValueError('explicit_flat_patch_set_required')
    hashes={m.name:hashlib.sha256(patch.extractfile(m).read()).hexdigest() for m in members}
    patch.extractall(candidate,members=members)
    isolated=tests(candidate,'candidate')
    if isolated['exit']!=0:raise ValueError('candidate_tests_failed_runtime_not_updated')
    audit_output=base/'apple-r16-recorded-outcome-regression-20261018-01.json'
    replay=subprocess.run([sys.executable,str(base/'reaudit_outcome_r16_20261018.py'),'--code',str(candidate),'--output',str(audit_output)])
    if replay.returncode:raise ValueError('recorded_regression_failed_runtime_not_updated')
    for key,root in roots.items():
        for name in names:
            if (sha(root/name) if (root/name).exists() else None)!=before[key][name]:raise ValueError('source_changed_since_validation')
    with tarfile.open(backup,'x:gz') as old:
        for key,root in roots.items():
            for name in names:
                if (root/name).exists():old.add(root/name,arcname=key+'/'+name)
    for root in roots.values():
        patch.extractall(root,members=members)
        if any(sha(root/name)!=digest for name,digest in hashes.items()):raise ValueError('deployed_hash_mismatch')
runtime=tests(roots['runtime'],'runtime')
with (base/'apple-outcome-r16-20261018-inputs.json').open('x') as stream:
    preflight=subprocess.run([sys.executable,str(roots['runtime']/'apple_transfer_automation.py'),'--check-inputs'],stdout=stream,stderr=subprocess.STDOUT)
report=dict(status='deployed_tests_passed' if runtime['exit']==0 and preflight.returncode==0 else 'deployed_validation_failed',
    revision='outcome-r16',branch=branch,archive=str(archive),sha256=sha(archive),backup=str(backup),
    copies_hash_verified=3,hashes=hashes,original_code_hashes=before,candidate_tests=isolated,runtime_tests=runtime,
    recorded_regression=str(audit_output),input_preflight_exit=preflight.returncode,
    robot_commands_sent=False,new_live_trial_performed=False,whole_uninterrupted_transfer_verified=False,
    original_recorded_results_unchanged=True,navigation_tolerances_changed=False,
    model_map_or_unity_changed=False,task_finished_sent=False,git_commit_or_push_performed=False,
    acceptance='Actual confirmed grasp and actual stable placement; planned XY and exact historical joints are not task-success criteria. Cleanup state reported separately.')
with report_path.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in report.items() if k not in ('hashes','original_code_hashes')}),flush=True)
raise SystemExit(0 if report['status']=='deployed_tests_passed' else 1)
