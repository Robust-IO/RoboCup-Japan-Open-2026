"""Can-only source identity640 + duplicate suppression. No robot actions."""
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
code=roots['runtime'];prefix='can-source640-v1-20261023';validation=base/(prefix+'-validation');validation.mkdir(exist_ok=False)
candidate=validation/'candidate';patch=base/(prefix+'-patch.tar.gz')
changed=['run_canned_grasp_trial.py','OBJECT_TRANSFER.txt'];added=['can_identity_selection.py','test_can_identity_selection.py'];names=changed+added
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
if sha(patch)!=args.sha:raise ValueError('patch_hash_mismatch')
expected=read(base/'can-cap-axis-v1-20261023-deployment.json')['hashes']
frozen={label:{p.relative_to(code).as_posix():sha(root/p.relative_to(code)) for p in code.rglob('*')
    if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')
    and p.name not in changed and (root/p.relative_to(code)).is_file()} for label,root in roots.items()}
cfg=read(code/'config/apple_transfer_auto_v1.json');registry=read(Path(cfg['registry']));model_sha=sha(Path(registry['weights']))
for root in roots.values():
    if any(sha(root/n)!=expected[n] for n in changed) or any((root/n).exists() for n in added):raise ValueError('unexpected_target_source')
def unchanged():
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
    if sha(Path(registry['weights']))!=model_sha:raise ValueError('model_changed')
shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(patch) as archive:
    if set(archive.getnames())!=set(names):raise ValueError('unexpected_archive_files')
    for m in archive.getmembers():
        if not m.isfile() or Path(m.name).name!=m.name:raise ValueError('unsafe_member')
        with (candidate/m.name).open('wb' if m.name in changed else 'xb') as f:f.write(archive.extractfile(m).read())
for n in names:
    if n.endswith('.py'):ast.parse((candidate/n).read_text())
env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts'))
with (validation/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,env=env,
        stdout=f,stderr=subprocess.STDOUT,timeout=180)
output=(validation/'tests.log').read_text();print(output[-2200:],flush=True)
if proc.returncode:raise ValueError('candidate_tests_failed')
count=int(re.search(r'Ran (\d+) tests',output).group(1));sys.path.insert(0,str(candidate))
from can_identity_selection import confirmed_cans
audit=read(base/'can-auto-03-identity-audit/historical640-source-regression.json')
if not audit['all_sources_have_high_confirmed_can'] or audit['model_sha256']!=model_sha:raise ValueError('actual_source_model_replay_required')
for row in audit['rows']:
    detections=[dict(item,name='canned_juice') for item in row['cans']]
    if len(confirmed_cans(detections))!=1:raise ValueError('real_frame_not_uniquely_confirmed')
from apple_transfer_automation import RobotLock
with RobotLock():
    unchanged()
    with tarfile.open(validation/'previous-files.tar.gz','x:gz') as archive:
        for label,root in roots.items():
            for n in changed:archive.add(root/n,arcname=label+'/'+n)
    hashes={n:sha(candidate/n) for n in names}
    for root in roots.values():
        if any(sha(root/n)!=expected[n] for n in changed):raise ValueError('concurrent_entry_change')
        for n in names:
            pending=root/(n+'.identity-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n)
        if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('copy_mismatch')
    unchanged()
    result=dict(status='can_source640_identity_deployed',tests=count,files=names,hashes=hashes,
        model_sha256=model_sha,confidence_threshold=.8,confidence_threshold_lowered=False,
        actual_source_frames_replayed=len(audit['rows']),all_previous_confirmations_preserved=True,
        no_model_retraining=True,apple_worker_and_navigation_unchanged=True,robot_commands_sent=False)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
