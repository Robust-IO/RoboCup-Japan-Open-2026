"""Verify then deploy publication-time observation renewal. No robot actions."""
import argparse
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

p=argparse.ArgumentParser();p.add_argument('--sha',required=True);args=p.parse_args()
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
roots=dict(runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),training=repo/'training',module=repo/'training/automation/apple_transfer_v1')
code=roots['runtime'];prefix='can-release-timing-v1-20261023';validation=base/(prefix+'-validation');validation.mkdir(exist_ok=False)
candidate=validation/'candidate';archive_path=base/(prefix+'-patch.tar.gz')
names=['run_canned_place_trial.py','can_release_timing.py','test_can_release_timing.py']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
if sha(archive_path)!=args.sha:raise ValueError('patch_hash_mismatch')
expected=read(base/'can-cap-axis-v1-20261023-deployment.json')['hashes']['run_canned_place_trial.py']
frozen={label:{p.relative_to(code).as_posix():sha(root/p.relative_to(code)) for p in code.rglob('*')
    if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')
    and p.name!='run_canned_place_trial.py' and (root/p.relative_to(code)).is_file()} for label,root in roots.items()}
for root in roots.values():
    if sha(root/names[0])!=expected or any((root/n).exists() for n in names[1:]):raise ValueError('unexpected_target_source')
def unchanged():
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
unchanged();shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(archive_path) as archive:
    if set(archive.getnames())!=set(names):raise ValueError('unexpected_archive_files')
    for member in archive.getmembers():
        if not member.isfile() or Path(member.name).name!=member.name:raise ValueError('unsafe_member')
        with (candidate/member.name).open('wb' if member.name==names[0] else 'xb') as f:f.write(archive.extractfile(member).read())
for n in names:ast.parse((candidate/n).read_text())
env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts'))
with (validation/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,env=env,
        stdout=f,stderr=subprocess.STDOUT,timeout=180)
output=(validation/'tests.log').read_text();print(output[-2500:],flush=True)
if proc.returncode:raise ValueError('candidate_tests_failed')
count=int(re.search(r'Ran (\d+) tests',output).group(1))
sys.path.insert(0,str(candidate));from can_release_timing import reviewed_plan
context=read(base/'can-reviewed-carry-place-20261023-01/held-context.json')
plan=reviewed_plan(base/'can-reviewed-carry-place-20261023-01/place',context['run_id'],context['run_root'])
from apple_transfer_automation import RobotLock
with RobotLock():
    unchanged()
    with tarfile.open(validation/'previous-entry.tar.gz','x:gz') as archive:
        for label,root in roots.items():archive.add(root/names[0],arcname=label+'/'+names[0])
    hashes={n:sha(candidate/n) for n in names}
    for root in roots.values():
        if sha(root/names[0])!=expected:raise ValueError('concurrent_entry_change')
        for n in names:
            pending=root/(n+'.release-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n)
        if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('copy_mismatch')
    unchanged()
    result=dict(status='can_release_timing_renewal_deployed',tests=count,files=names,hashes=hashes,
        source_age_limit_s=.4,extra_joint_wait_removed=True,new_observation_retry_not_action_retry=True,
        actual_failed_pre_release_context_validated=True,robot_commands_sent=False)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
