"""Validate a separate D-drive candidate; no runtime writes or robot commands."""
import fcntl
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
archive=base/'apple-near-geometry-r12b-20261018-patch.tar.gz'
candidate=base/'apple-near-geometry-r12b-candidate-20261018-01'
report_path=base/'apple-near-geometry-r12b-20261018-candidate-validation.json'
names={'near_apple_geometry.py','apple_tracking_observer.py','capture_grasp_scene.py','apple_approach_lease.py',
    'verify_apple_lift_sequence.py','audit_preclose_identity_lift_follow.py','apple_transfer_auto_backend.py',
    'apple_transfer_auto_plan.py','config/apple_transfer_auto_v1.json','run_apple_close_lift_trial.py',
    'test_near_apple_geometry.py','test_apple_transfer_auto_backend.py','test_run_apple_close_lift_trial.py',
    'replay_near_apple_geometry.py','APPLE_TRANSFER_AUTOMATION.txt'}
roots=dict(runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),
    training=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training'),
    module=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/automation/apple_transfer_v1'))
lock=open('/tmp/handyman-apple-transfer-automation.lock','a+')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
if sha(archive)!='ccd7e68ee8a0967a0c4105dbe1dafd611fc594babf6050421daa097a0408cd14':raise ValueError('patch_hash_mismatch')
if report_path.exists():raise ValueError('candidate_validation_report_exists')
before={key:{name:sha(root/name) if (root/name).exists() else None for name in names} for key,root in roots.items()}
for name in names:
    if len({values[name] for values in before.values() if values[name] is not None})>1:
        raise ValueError('preexisting_code_copy_difference:'+name)
shutil.copytree(roots['runtime'],candidate,ignore=shutil.ignore_patterns('__pycache__'))
with tarfile.open(archive,'r:gz') as patch:
    members=patch.getmembers()
    if len(members)!=len(names) or {m.name for m in members}!=names or any(not m.isfile() for m in members):
        raise ValueError('explicit_patch_set_required')
    patch.extractall(candidate,members=members)
tests_log=base/'apple-near-geometry-r12b-20261018-candidate-tests.log'
with tests_log.open('x') as stream:
    tests=subprocess.run([sys.executable,'-m','unittest','discover','-p','test_*.py','-v'],
        cwd=candidate,stdout=stream,stderr=subprocess.STDOUT)
test_text=tests_log.read_text();count=re.findall(r'Ran (\d+) tests?',test_text)
print('\n'.join(test_text.splitlines()[-6:]),flush=True)
replay_exit=None;preflight_exit=None
replay=base/'apple-near-geometry-r12b-recorded-20261018-01'
if tests.returncode==0:
    with (base/'apple-near-geometry-r12b-20261018-inputs.json').open('x') as stream:
        preflight_exit=subprocess.run([sys.executable,str(candidate/'apple_transfer_automation.py'),'--check-inputs'],stdout=stream,stderr=subprocess.STDOUT).returncode
    with (base/'apple-near-geometry-r12b-20261018-replay.log').open('x') as stream:
        replay_exit=subprocess.run([sys.executable,str(candidate/'replay_near_apple_geometry.py'),'--recorded-only',
            '--recording',str(base/'automatic-apple-r11b-20261018-01/source-observe'),
            '--environment','/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/src/handyman_rebuild_ros2/config/environments/layout_a.yaml',
            '--output',str(replay)],stdout=stream,stderr=subprocess.STDOUT).returncode
unchanged=all((sha(root/name) if (root/name).exists() else None)==before[key][name]
              for key,root in roots.items() for name in names)
okay=tests.returncode==0 and preflight_exit==0 and replay_exit==0 and unchanged
report=dict(status='isolated_candidate_tests_and_recorded_replay_passed' if okay else 'candidate_validation_failed',
    candidate=str(candidate),patch_sha256=sha(archive),original_code_hashes=before,original_runtime_sources_unchanged=unchanged,
    tests=int(count[-1]) if count else None,test_exit=tests.returncode,tests_log=str(tests_log),
    preflight_exit=preflight_exit,replay_exit=replay_exit,replay_result=str(replay/'result.json'),
    robot_commands_sent=False,live_motion_validated=False,runtime_updated=False)
with report_path.open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='original_code_hashes'}),flush=True)
if replay_exit==0:
    result=json.loads((replay/'result.json').read_text())
    print(json.dumps({k:result[k] for k in ('status','frames','geometry_frames','no_apple_detector_frames_rescued','first_handoff_frame','last_frame','original_recordings_and_failed_result_hashes_unchanged')}),flush=True)
raise SystemExit(0 if okay else 1)
