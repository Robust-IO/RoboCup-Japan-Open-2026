"""Backed-up code/config deploy, tests and recorded-only replay. NO robot actions."""
import fcntl
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
archive=base/'apple-placement-policy-r11b-20261018-patch.tar.gz'
backup=base/'apple-before-placement-policy-r11b-20261018.tar.gz'
roots=dict(runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),
    training=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training'),
    module=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/automation/apple_transfer_v1'))
names={'config/apple_transfer_auto_v1.json','apple_transfer_auto_plan.py','apple_transfer_auto_backend.py',
       'reviewed_carry_adjustment.py','motion_speed.py','tabletop_support.py','table_release_gate.py',
       'replay_placement_policy_r11.py','test_motion_speed.py','test_reviewed_carry_adjustment.py',
       'test_tabletop_support.py','test_apple_transfer_auto_backend.py','test_segmented_apple_placement.py','APPLE_TRANSFER_AUTOMATION.txt'}
lock=open('/tmp/handyman-apple-transfer-automation.lock','a+')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
if hashlib.sha256(archive.read_bytes()).hexdigest()!='a26d90edfbc05173331179dc96290a003e8c0b5c38c40afded0ad4dc7b4d73a6':
    raise ValueError('patch_hash_mismatch')
with tarfile.open(archive,'r:gz') as patch:
    members=patch.getmembers()
    if len(members)!=14 or {m.name for m in members}!=names or any(not m.isfile() for m in members):
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
log=base/'apple-placement-policy-r11b-20261018-tests.log'
with log.open('x') as stream:
    tests=subprocess.run([sys.executable,'-m','unittest','discover','-p','test_*.py','-v'],
        cwd=roots['runtime'],stdout=stream,stderr=subprocess.STDOUT)
print('\n'.join(log.read_text().splitlines()[-5:]),flush=True)
recorded=base/'apple-placement-policy-r11b-recorded-20261018-01'
replay_exit=None;preflight_exit=None
if tests.returncode==0:
    with (base/'apple-placement-policy-r11b-20261018-inputs.json').open('x') as f:
        check=subprocess.run([sys.executable,str(roots['runtime']/'apple_transfer_automation.py'),'--check-inputs'],stdout=f)
        preflight_exit=check.returncode
    with (base/'apple-placement-policy-r11b-20261018-replay.log').open('x') as f:
        replay=subprocess.run([sys.executable,str(roots['runtime']/'replay_placement_policy_r11.py'),
            '--recording',str(base/'apple-navigation-feedback-r10-live-place-20261018-01/destination-outside'),
            '--verified-held',str(base/'apple-gate-pruning-live-carry-place-20261018-01/verified-held-geometry.json'),
            '--config',str(roots['runtime']/'config/apple_transfer_auto_v1.json'),
            '--output',str(recorded)],stdout=f,stderr=subprocess.STDOUT)
        replay_exit=replay.returncode
okay=tests.returncode==0 and preflight_exit==0 and replay_exit==0
report=dict(status='deployed_tests_and_recorded_replanning_passed' if okay else 'deployed_validation_failed',
    revision='20261018-placement-policy-r11b',backup=str(backup),hashes=expected,
    tests_log=str(log),test_exit=tests.returncode,preflight_exit=preflight_exit,replay_exit=replay_exit,
    replay_result=str(recorded/'result.json'),single_placement_translation_cap_m=None,
    edge_outer_gap_m=.03,edge_uncertainty_m=.01,robot_commands_sent=False,
    new_policy_live_test_performed=False,navigation_tolerances_changed=False)
with (base/'apple-placement-policy-r11b-20261018-deployment.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report),flush=True)
if replay_exit==0:
    result=json.loads((recorded/'result.json').read_text())
    print(json.dumps(dict(recorded_frames=result['frames'],approach_distances_m=[r['planned_approach_distance_m'] for r in result['rows']],
        source_unchanged=result['original_recordings_and_failed_result_unchanged'],robot_commands_sent=False)),flush=True)
raise SystemExit(0 if okay else 1)
