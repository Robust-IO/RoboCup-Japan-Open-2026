"""Offline validation and atomic three-copy deployment of the pepper adapter."""
import argparse
import ast
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--sha',required=True);args=p.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
roots={'runtime':Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),
    'training':repo/'training','module':repo/'training/automation/apple_transfer_v1'}
code=roots['runtime'];prefix='pepper-transfer-v1-20261024';v=base/(prefix+'-validation')
new=['pepper_geometry.py','pepper_hold.py','pepper_place.py','pepper_runtime.py','pepper_transfer_automation.py',
    'replay_pepper_reference.py','test_pepper_geometry.py','test_pepper_transfer.py']
changed=['canned_grasp_runtime.py','manipulation_profiles.py','config/manipulation_objects_v1.json',
    'object_surface_geometry.py','generic_transfer_template.py','run_object_transfer.py',
    'inspect_manipulation_capture.py','OBJECT_TRANSFER.txt']
names=new+changed;sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();read=lambda p:json.loads(p.read_text())
previous=read(base/'canned-initial-return-continuous-success-progress-20261024-09.json')
patch=base/(prefix+'-patch.tar.gz')
if sha(patch)!=args.sha:raise ValueError('patch_hash_mismatch')
for root in roots.values():
    if any(sha(root/n)!=digest for n,digest in previous['latest_expected_hashes'].items()):raise ValueError('tested_source_changed')
    if any((root/n).exists() for n in new):raise ValueError('new_pepper_source_already_exists')
old={n:sha(code/n) for n in changed}
for root in roots.values():
    if any(sha(root/n)!=digest for n,digest in old.items()):raise ValueError('existing_code_copies_differ')
allowed={(root/n).resolve() for root in roots.values() for n in names}
frozen={label:{p.relative_to(root).as_posix():sha(p) for p in root.rglob('*') if p.is_file()
    and '__pycache__' not in p.parts and not p.name.endswith('.pyc') and p.resolve() not in allowed}
    for label,root in roots.items()}
v.mkdir(exist_ok=False);candidate=v/'candidate'
shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(patch) as archive:
    if set(archive.getnames())!=set(names):raise ValueError('unexpected_patch_files')
    for member in archive.getmembers():
        if not member.isfile() or '..' in Path(member.name).parts or Path(member.name).is_absolute():raise ValueError('unsafe_archive')
        target=candidate/member.name;target.parent.mkdir(exist_ok=True,parents=True)
        target.write_bytes(archive.extractfile(member).read())
for n in names:
    if n.endswith('.py'):ast.parse((candidate/n).read_text())
old_objects=read(code/'config/manipulation_objects_v1.json')['objects']
new_objects=read(candidate/'config/manipulation_objects_v1.json')['objects']
if any(old_objects[n]!=new_objects[n] for n in old_objects):raise ValueError('existing_object_profiles_changed')
with (v/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),stdout=f,stderr=subprocess.STDOUT,timeout=180)
log=(v/'tests.log').read_text();print(log[-2000:],flush=True)
if proc.returncode:raise ValueError('pepper_runtime_regression_failed')
tests=int(re.search(r'Ran (\d+) tests',log).group(1));sys.path.insert(0,str(candidate))
reference=base/'pepper-offline-reference-v1-20261024'
if sha(reference/'frames.tar.gz')!='d8923507eacfd1c01a8273516a8049f4dd6b3ec758cf01f02ac7ff9342947711':
    raise ValueError('actual_reference_archive_changed')
frames=reference/'frames';frames.mkdir(exist_ok=True)
with tarfile.open(reference/'frames.tar.gz') as archive:
    for member in archive.getmembers():
        if not member.isfile() or len(Path(member.name).parts)!=2 or '..' in Path(member.name).parts or Path(member.name).is_absolute():
            raise ValueError('unsafe_reference_frame')
        target=frames/member.name;target.parent.mkdir(exist_ok=True)
        content=archive.extractfile(member).read()
        if target.exists() and target.read_bytes()!=content:raise ValueError('existing_reference_frame_changed')
        if not target.exists():target.write_bytes(content)
from replay_pepper_reference import replay
from apple_transfer_automation import RobotLock,check_inputs
replay_result=replay(reference)
if replay_result['frames']!=6:raise ValueError('six_real_pepper_frames_required')
(v/'real-rgbd-replay.json').write_text(json.dumps(replay_result,indent=2,allow_nan=False))
cfg=read(code/'config/apple_transfer_auto_v1.json');input_audit=check_inputs(cfg)
with RobotLock():
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
        if any(sha(root/n)!=digest for n,digest in old.items()):raise ValueError('concurrent_edit')
    with tarfile.open(v/'previous-files.tar.gz','x:gz') as archive:
        for label,root in roots.items():
            for n in changed:archive.add(root/n,arcname=label+'/'+n)
    hashes={n:sha(candidate/n) for n in names}
    for root in roots.values():
        for n in names:
            pending=root/(n+'.pepper-pending');pending.parent.mkdir(exist_ok=True,parents=True)
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n)
        if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('copy_mismatch')
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
    result=dict(status='ground_pepper_adapter_deployed_offline_verified_live_manipulation_pending',
        tests_passed=tests,hashes=hashes,previous_hashes=old,
        actual_saved_rgbd_frames_verified=replay_result['frames'],height_range_m=replay_result['height_range_m'],
        body_radius_range_m=replay_result['body_radius_range_m'],
        model_and_map_audit=input_audit,original_apple_can_profiles_unchanged=True,
        original_navigation_files_unchanged=True,robot_commands_sent=False,unity_required_during_development=False,
        live_grasp_verified=False,live_carry_verified=False,live_placement_verified=False,
        live_entry='run_object_transfer.sh --object ground_pepper',
        current_robot_state_changed_by_deployment=False)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k not in ('hashes','previous_hashes')}),flush=True)
