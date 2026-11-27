"""Deploy the independently validated patch, with per-copy backup; no motion."""
import fcntl
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tarfile

base=Path('/__HANDYMAN_PACKAGE__/runtime')
archive=base/'apple-near-geometry-r12b-20261018-patch.tar.gz'
backup=base/'apple-before-near-geometry-r12b-20261018.tar.gz'
validation=json.loads((base/'apple-near-geometry-r12b-20261018-candidate-validation.json').read_text())
roots=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),
    training=Path('/__HANDYMAN_PACKAGE__/project/training'),
    module=Path('/__HANDYMAN_PACKAGE__/project/training/automation/apple_transfer_v1'))
names={'near_apple_geometry.py','apple_tracking_observer.py','capture_grasp_scene.py','apple_approach_lease.py',
    'verify_apple_lift_sequence.py','audit_preclose_identity_lift_follow.py','apple_transfer_auto_backend.py',
    'apple_transfer_auto_plan.py','config/apple_transfer_auto_v1.json','run_apple_close_lift_trial.py',
    'test_near_apple_geometry.py','test_apple_transfer_auto_backend.py','test_run_apple_close_lift_trial.py',
    'replay_near_apple_geometry.py','APPLE_TRANSFER_AUTOMATION.txt'}
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
lock=open('/tmp/handyman-apple-transfer-automation.lock','a+')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
if (validation['status']!='isolated_candidate_tests_and_recorded_replay_passed'
        or validation['patch_sha256']!=sha(archive)
        or sha(archive)!='ccd7e68ee8a0967a0c4105dbe1dafd611fc594babf6050421daa097a0408cd14'):
    raise ValueError('validated_exact_patch_required')
branch=subprocess.check_output(['git','-C','/__HANDYMAN_PACKAGE__/project','branch','--show-current'],text=True).strip()
if branch!='feat/handyman-phase5-grasping':raise ValueError('unexpected_active_branch:'+branch)
for key,root in roots.items():
    for name in names:
        actual=sha(root/name) if (root/name).exists() else None
        if actual!=validation['original_code_hashes'][key][name]:raise ValueError('source_changed_since_validation:'+key+':'+name)
with tarfile.open(archive,'r:gz') as patch:
    members=patch.getmembers()
    if len(members)!=len(names) or {m.name for m in members}!=names or any(not m.isfile() for m in members):
        raise ValueError('explicit_patch_file_set_required')
    expected={m.name:hashlib.sha256(patch.extractfile(m).read()).hexdigest() for m in members}
    if any(sha(Path(validation['candidate'])/name)!=digest for name,digest in expected.items()):
        raise ValueError('candidate_changed_since_validation')
    with tarfile.open(backup,'x:gz') as old:
        for key,root in roots.items():
            for name in names:
                if (root/name).exists():old.add(root/name,arcname=key+'/'+name)
    for root in roots.values():
        patch.extractall(root,members=members)
        if any(sha(root/name)!=digest for name,digest in expected.items()):raise ValueError('deployed_hash_mismatch')
test_log=base/'apple-near-geometry-r12b-20261018-runtime-tests.log'
with test_log.open('x') as stream:
    tests=subprocess.run([sys.executable,'-m','unittest','discover','-p','test_*.py','-v'],
        cwd=roots['runtime'],stdout=stream,stderr=subprocess.STDOUT)
text=test_log.read_text();count=re.findall(r'Ran (\d+) tests?',text)
print('\n'.join(text.splitlines()[-6:]),flush=True)
preflight_exit=None
if tests.returncode==0:
    with (base/'apple-near-geometry-r12b-20261018-runtime-inputs.json').open('x') as stream:
        preflight_exit=subprocess.run([sys.executable,str(roots['runtime']/'apple_transfer_automation.py'),'--check-inputs'],
            stdout=stream,stderr=subprocess.STDOUT).returncode
report=dict(status='deployed_tests_and_recorded_replay_passed' if tests.returncode==0 and preflight_exit==0 else 'deployed_validation_failed',
    implementation_revision='20261018-near-geometry-r12b',branch=branch,patch=str(archive),patch_sha256=sha(archive),
    backup=str(backup),copies_hash_verified=3,hashes=expected,
    runtime_tests=int(count[-1]) if count else None,test_exit=tests.returncode,preflight_exit=preflight_exit,
    runtime_tests_log=str(test_log),recorded_replay=validation['replay_result'],
    near_geometry_enabled=True,remaining_contact_gap_m=.15,model_or_map_changed=False,
    navigation_tolerances_changed=False,robot_commands_sent=False,live_motion_validated=False,
    uninterrupted_full_transfer_verified=False,git_commit_or_push_performed=False)
with (base/'apple-near-geometry-r12b-20261018-deployment.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='hashes'}),flush=True)
raise SystemExit(0 if report['status']=='deployed_tests_and_recorded_replay_passed' else 1)
