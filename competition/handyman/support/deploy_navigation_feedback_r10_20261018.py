"""Verified file-only r10 deploy with backups; no robot services or imports."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

base=Path('/__HANDYMAN_PACKAGE__/runtime')
archive=base/'apple-navigation-feedback-r10-20261018-patch.tar.gz'
backup=base/'apple-before-navigation-feedback-r10-20261018.tar.gz'
roots=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),
    training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
names={'navigation_feedback_gate.py','navigate_carry_trial.py','test_navigation_feedback_gate.py',
       'probe_navigation_latency.py','APPLE_TRANSFER_AUTOMATION.txt'}
if hashlib.sha256(archive.read_bytes()).hexdigest()!='5e0da9988b1a28e344fda8317ae4efb10c2f0d9c4d04c690dfbc66b40cf2800e':
    raise ValueError('patch_hash_mismatch')
with tarfile.open(archive,'r:gz') as patch:
    members=patch.getmembers()
    if len(members)!=5 or {m.name for m in members}!=names or any(not m.isfile() for m in members):
        raise ValueError('unexpected_patch_members')
    for name in names:
        hashes={hashlib.sha256((root/name).read_bytes()).hexdigest() for root in roots.values() if (root/name).exists()}
        if len(hashes)>1:raise ValueError('preexisting_copy_difference:'+name)
    with tarfile.open(backup,'x:gz') as old:
        for key,root in roots.items():
            for member in members:
                if (root/member.name).exists():old.add(root/member.name,arcname=key+'/'+member.name)
    expected={m.name:hashlib.sha256(patch.extractfile(m).read()).hexdigest() for m in members}
    for root in roots.values():
        patch.extractall(root,members=members)
        if any(hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest for name,digest in expected.items()):
            raise ValueError('installed_hash_mismatch')
log=base/'apple-navigation-feedback-r10-20261018-tests.log'
with log.open('x') as stream:
    test=subprocess.run(['/usr/bin/python3','-m','unittest','discover','-p','test_*.py','-v'],
        cwd=roots['runtime'],stdout=stream,stderr=subprocess.STDOUT)
report=dict(status='deployed_and_tested' if test.returncode==0 else 'deployed_tests_failed',
    revision='20261018-navigation-feedback-r10',backup=str(backup),hashes=expected,
    test_log=str(log),test_exit=test.returncode,robot_commands_sent=False,
    sample_age_limit_s=.7,new_feedback_acquisition_budget_s=.15,
    existing_nav2_goal_runs_during_acquisition=True,navigation_tolerances_changed=False)
with (base/'apple-navigation-feedback-r10-20261018-deployment.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report),flush=True)
print('\n'.join(log.read_text().splitlines()[-5:]),flush=True)
raise SystemExit(test.returncode)
