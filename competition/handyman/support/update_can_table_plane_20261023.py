"""Can-only table-plane fallback update; no robot actions."""
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
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
roots=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),training=repo/'training',module=repo/'training/automation/apple_transfer_v1')
code=roots['runtime'];prefix='can-table-plane-v1-20261023';validation=base/(prefix+'-validation');validation.mkdir(exist_ok=False)
candidate=validation/'candidate';patch=base/(prefix+'-patch.tar.gz')
names=['run_canned_place_trial.py','can_destination_surface.py','test_can_destination_surface.py']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
if sha(patch)!=args.sha:raise ValueError('patch_hash_mismatch')
expected=read(base/'can-release-timing-v1-20261023-deployment.json')['hashes'][names[0]]
frozen={label:{p.relative_to(code).as_posix():sha(root/p.relative_to(code)) for p in code.rglob('*')
    if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')
    and p.name!=names[0] and (root/p.relative_to(code)).is_file()} for label,root in roots.items()}
for root in roots.values():
    if sha(root/names[0])!=expected or any((root/n).exists() for n in names[1:]):raise ValueError('unexpected_target_source')
def unchanged():
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(patch) as archive:
    if set(archive.getnames())!=set(names):raise ValueError('unexpected_archive_files')
    for m in archive.getmembers():
        if not m.isfile() or Path(m.name).name!=m.name:raise ValueError('unsafe_member')
        with (candidate/m.name).open('wb' if m.name==names[0] else 'xb') as f:f.write(archive.extractfile(m).read())
for n in names:ast.parse((candidate/n).read_text())
env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts'))
with (validation/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,env=env,
        stdout=f,stderr=subprocess.STDOUT,timeout=180)
output=(validation/'tests.log').read_text();print(output[-2300:],flush=True)
if proc.returncode:raise ValueError('candidate_tests_failed')
count=int(re.search(r'Ran (\d+) tests',output).group(1))
sys.path.insert(0,str(candidate));import numpy as np
from can_destination_surface import measure
from canned_place_geometry import make_plan
from apple_transfer_automation import RobotLock
cfg=read(code/'config/apple_transfer_auto_v1.json');table=next(x for x in read(Path(cfg['semantic_map']))['destinations'] if x['id']==cfg['destination_id'])
reference=read(base/'can-auto-live-20261023-02/held-context.json');recorded=[]
failed=base/'can-auto-live-20261023-02/place/result.json';failed_sha=sha(failed)
for path in sorted((failed.parent/'captures').glob('*destination-rejected')):
    snapshot=read(path/'snapshot.json');depth=np.load(path/'depth_m.npy',allow_pickle=False)
    plane=measure(depth,snapshot,table);plan=make_plan(depth,snapshot,table,plane,reference,cfg,'table-plane-replay')
    if plane['residual_p95_m']>.004 or not plan['table_edge_safety']['safe']:raise ValueError('actual_table_replay_not_valid')
    recorded.append(dict(capture=str(path),height_m=plane['table_height_map_m'],residual_p95_m=plane['residual_p95_m'],
        inlier_fraction=plane.get('actual_plane_inlier_fraction'),removed=plane.get('near_height_outliers_removed'),release_xy=plan['release_xy_map_m']))
if len(recorded)!=5:raise ValueError('all_five_failed_frames_required')
prior=read(base/'can-destination-measured-20261023-01/result.json')
for row in prior['rows']:
    path=Path(row['capture']);plane=measure(np.load(path/'depth_m.npy',allow_pickle=False),read(path/'snapshot.json'),table)
    if abs(plane['table_height_map_m']-row['plane']['table_height_map_m'])>1e-8:raise ValueError('previous_successful_plane_changed')
if sha(failed)!=failed_sha:raise ValueError('failed_verdict_changed')
with RobotLock():
    unchanged()
    with tarfile.open(validation/'previous-entry.tar.gz','x:gz') as archive:
        for label,root in roots.items():archive.add(root/names[0],arcname=label+'/'+names[0])
    hashes={n:sha(candidate/n) for n in names}
    for root in roots.values():
        if sha(root/names[0])!=expected:raise ValueError('concurrent_entry_change')
        for n in names:
            pending=root/(n+'.table-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n)
        if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('copy_mismatch')
    unchanged()
    result=dict(status='can_robust_dominant_table_plane_deployed',tests=count,files=names,hashes=hashes,
        actual_failed_frame_replays=recorded,prior_successful_frames_unchanged=3,
        original_apple_estimator_unchanged=True,residual_limit_silently_relaxed=False,robot_commands_sent=False)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
