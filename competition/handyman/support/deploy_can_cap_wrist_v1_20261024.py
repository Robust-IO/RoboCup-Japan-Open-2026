"""Validate and deploy the optional deeper-can seating experiment only."""
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
code=roots['runtime'];prefix='can-cap-wrist-v1-20261024';v=base/(prefix+'-validation')
names=['tracked_can_cap_geometry.py','test_tracked_can_cap_geometry.py','OBJECT_TRANSFER.txt']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();read=lambda p:json.loads(p.read_text())
expected=dict(read(base/'canned-upright-reset-required-progress-20261024.json')['latest_expected_hashes'],
    **read(base/'can-extra-seat35-20261024-deployment.json')['hashes'])
patch=base/(prefix+'-patch.tar.gz')
if sha(patch)!=args.sha:raise ValueError('patch_hash_mismatch')
for root in roots.values():
    if any(sha(root/n)!=digest for n,digest in expected.items()):raise ValueError('tested_source_changed')
old={n:sha(code/n) for n in names};allowed={(root/n).resolve() for root in roots.values() for n in names}
frozen={label:{p.relative_to(root).as_posix():sha(p) for p in root.rglob('*') if p.is_file()
    and '__pycache__' not in p.parts and not p.name.endswith('.pyc') and p.resolve() not in allowed}
    for label,root in roots.items()}
v.mkdir(exist_ok=False);candidate=v/'candidate'
shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(patch) as archive:
    if set(archive.getnames())!=set(names):raise ValueError('unexpected_patch_files')
    for member in archive.getmembers():
        if not member.isfile() or Path(member.name).name!=member.name:raise ValueError('unsafe_archive')
        (candidate/member.name).write_bytes(archive.extractfile(member).read())
for n in names:
    if n.endswith('.py'):ast.parse((candidate/n).read_text())
with (v/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),stdout=f,stderr=subprocess.STDOUT,timeout=180)
out=(v/'tests.log').read_text();print(out[-1800:],flush=True)
if proc.returncode:raise ValueError('candidate_tests_failed')
count=int(re.search(r'Ran (\d+) tests',out).group(1));sys.path.insert(0,str(candidate))
import numpy as np
from canned_grasp_geometry import contact_offset,closure_error
from canned_grasp_geometry import measure_near,predicted_before_contact
from apple_transfer_automation import RobotLock
baseline=np.array([.07948608136287283,.012144502421366304,0.])
if not np.allclose(contact_offset(),baseline,atol=1e-12):raise ValueError('default_calibration_changed')
if not np.allclose(contact_offset(.035),baseline-[.035,0,0],atol=1e-12):raise ValueError('wrong_seating_direction')
import cv2
failed=base/'can-extra-seat35-live-20261024-02/grasp'
ref=read(failed/'locked-can-reference.json');replays=[]
for capture in sorted((failed/'captures').glob('rejected-geometry-*')):
    snap=read(capture/'snapshot.json');expected_center=predicted_before_contact(ref,snap,'grasp')
    rgb=cv2.cvtColor(cv2.imread(str(capture/'rgb.png')),cv2.COLOR_BGR2RGB)
    m=measure_near(np.load(capture/'depth_m.npy'),snap,expected_center,ref['radius_m'],ref['height_m'],
        rgb=rgb,appearance=ref['source_appearance'])
    if m['tilt_deg']>1. or m['nearby_plane_points_excluded']<1:raise ValueError('real_wrist_failure_not_resolved')
    replays.append(dict(capture=str(capture),measurement=m))
if len(replays)!=5:raise ValueError('five_real_failure_frames_required')
(v/'actual-failed-frame-replay.json').write_text(json.dumps(replays,indent=2))
with RobotLock():
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
        if any(sha(root/n)!=digest for n,digest in old.items()):raise ValueError('concurrent_edit')
    with tarfile.open(v/'previous-files.tar.gz','x:gz') as archive:
        for label,root in roots.items():
            for n in names:archive.add(root/n,arcname=label+'/'+n)
    hashes={n:sha(candidate/n) for n in names}
    for root in roots.values():
        for n in names:
            pending=root/(n+'.seat35-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n)
        if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('copy_mismatch')
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
    result=dict(status='observed_cap_region_refinement_deployed',tests=count,hashes=hashes,previous_hashes=old,
        experiment_extra_seat_m=.035,default_extra_seat_m=0.,side_and_height_calibration_changed=False,
        record_postclose_pose_diagnostic_only=True,original_navigation_model_and_apple_unchanged=True,
        actual_failed_frames_replayed=len(replays),actual_replay_tilts_deg=[r['measurement']['tilt_deg'] for r in replays],
        robot_commands_sent=False,live_trial_still_required=True)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
