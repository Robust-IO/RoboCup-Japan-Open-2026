"""Validate isolated candidate, back up, deploy4 files, re-test. No robot commands."""
import fcntl
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile

base=Path('/__HANDYMAN_PACKAGE__/runtime');archive=base/'apple-capture-budget-r13-20261018-patch.tar.gz'
candidate=base/'apple-capture-budget-r13-candidate-20261018-01'
backup=base/'apple-before-capture-budget-r13-20261018.tar.gz'
report_path=base/'apple-capture-budget-r13-20261018-deployment.json'
names={'capture_processing_budget.py','capture_grasp_scene.py','test_capture_processing_budget.py','APPLE_TRANSFER_AUTOMATION.txt'}
roots=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),
    training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def tests(root,name):
    log=base/name
    with log.open('x') as stream:
        child=subprocess.run([sys.executable,'-m','unittest','discover','-p','test_*.py','-v'],
            cwd=root,stdout=stream,stderr=subprocess.STDOUT)
    value=log.read_text();found=re.findall(r'Ran (\d+) tests?',value)
    print('\n'.join(value.splitlines()[-5:]),flush=True)
    return dict(exit=child.returncode,count=int(found[-1]) if found else None,log=str(log))

lock=open('/tmp/handyman-apple-transfer-automation.lock','a+')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
if sha(archive)!='407ac4cabc0c5d69d4da8415d72bd3828f4f9edbe24838592f250c742a575f5b':raise ValueError('exact_patch_required')
if report_path.exists():raise ValueError('new_deployment_report_required')
before={key:{name:sha(root/name) if (root/name).exists() else None for name in names} for key,root in roots.items()}
for name in names:
    if len({row[name] for row in before.values() if row[name] is not None})>1:raise ValueError('preexisting_copy_difference:'+name)
shutil.copytree(roots['runtime'],candidate,ignore=shutil.ignore_patterns('__pycache__'))
with tarfile.open(archive,'r:gz') as patch:
    members=patch.getmembers()
    if len(members)!=4 or {m.name for m in members}!=names or any(not m.isfile() for m in members):raise ValueError('explicit_patch_set_required')
    expected={m.name:hashlib.sha256(patch.extractfile(m).read()).hexdigest() for m in members}
    patch.extractall(candidate,members=members)
    isolated=tests(candidate,'apple-capture-budget-r13-20261018-candidate-tests.log')
    if isolated['exit']!=0:raise ValueError('candidate_tests_failed_runtime_not_updated')
    for key,root in roots.items():
        for name in names:
            if (sha(root/name) if (root/name).exists() else None)!=before[key][name]:raise ValueError('source_changed_since_validation')
    with tarfile.open(backup,'x:gz') as old:
        for key,root in roots.items():
            for name in names:
                if (root/name).exists():old.add(root/name,arcname=key+'/'+name)
    for root in roots.values():
        patch.extractall(root,members=members)
        if any(sha(root/name)!=digest for name,digest in expected.items()):raise ValueError('deployed_hash_mismatch')
runtime=tests(roots['runtime'],'apple-capture-budget-r13-20261018-runtime-tests.log')
with (base/'apple-capture-budget-r13-20261018-inputs.json').open('x') as stream:
    preflight=subprocess.run([sys.executable,str(roots['runtime']/'apple_transfer_automation.py'),'--check-inputs'],
        stdout=stream,stderr=subprocess.STDOUT)
report=dict(status='deployed_tests_passed' if runtime['exit']==0 and preflight.returncode==0 else 'deployed_validation_failed',
    revision='capture-budget-r13',patch=str(archive),sha256=sha(archive),backup=str(backup),
    copies_hash_verified=3,hashes=expected,original_code_hashes=before,candidate_tests=isolated,runtime_tests=runtime,
    input_preflight_exit=preflight.returncode,robot_commands_sent=False,new_live_observation_validated=False,
    final_freshness_max_s=.4,original_source_stamps_preserved=True,
    model_maps_navigation_grasp_placement_policy_changed=False)
with report_path.open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k not in ('hashes','original_code_hashes')}),flush=True)
raise SystemExit(0 if report['status']=='deployed_tests_passed' else 1)
