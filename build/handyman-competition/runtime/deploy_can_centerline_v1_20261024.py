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
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
roots={'runtime':Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),
    'training':repo/'training','module':repo/'training/automation/apple_transfer_v1'}
code=roots['runtime'];prefix='can-centerline-v1-20261024';v=base/(prefix+'-validation')
names=['canned_grasp_geometry.py','can_approach_plan.py','test_canned_grasp_geometry.py','test_can_approach_plan.py','OBJECT_TRANSFER.txt']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();read=lambda p:json.loads(p.read_text())
expected=dict(read(base/'canned-upright-reset-required-progress-20261024.json')['latest_expected_hashes'],
    **read(base/'can-extra-seat35-20261024-deployment.json')['hashes'])
expected.update(read(base/'can-cap-wrist-v1-20261024-deployment.json')['hashes'])
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
from canned_grasp_geometry import aligned,CLOSE_TOLERANCE
from can_approach_plan import approach_plan
from apple_transfer_automation import RobotLock
audit=read(base/'can-centerline-stop-audit-20261024-01/result.json')
baseline=np.array([.07948608136287283,audit['finger_midpoint_base_offset_m'][1],0.])
if not np.allclose(contact_offset(),baseline,atol=1e-12):raise ValueError('default_calibration_changed')
if not np.allclose(contact_offset(.035),baseline-[.035,0,0],atol=1e-12):raise ValueError('wrong_seating_direction')
source=base/'can-extra-seat35-live-20261024-03/grasp/captures/preclose'
snap=read(source/'snapshot.json');measurement=read(source/'can-measurement.json')
error=closure_error(measurement['center_m'],snap,extra_seat_m=.035)
if abs(error[1])<=CLOSE_TOLERANCE[1] or aligned(measurement['center_m'],snap,extra_seat_m=.035):
    raise ValueError('archived_offcenter_can_not_rejected')
if approach_plan(error)['phase']!='clear_before_lateral_alignment':raise ValueError('would_sweep_fingers_sideways_at_contact')
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
    result=dict(status='can_true_finger_midline_alignment_deployed_not_live_verified',tests=count,hashes=hashes,previous_hashes=old,
        experiment_extra_seat_m=.035,default_extra_seat_m=0.,side_calibration_changed=True,height_calibration_changed=False,
        finger_midline_base_y_offset_m=float(baseline[1]),lateral_alignment_tolerance_m=float(CLOSE_TOLERANCE[1]),
        archived_offcenter_preclose_error_m=error.tolist(),
        actual_midline_audit=str(base/'can-centerline-stop-audit-20261024-01/result.json'),
        record_postclose_pose_diagnostic_only=True,original_navigation_model_and_apple_unchanged=True,
        robot_commands_sent=False,live_trial_still_required=True)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
