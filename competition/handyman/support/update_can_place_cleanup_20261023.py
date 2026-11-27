"""Use current pose during empty-hand cleanup; final RGBD proves placement."""
import argparse
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
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source=base/'run_canned_place_trial-v1.1.py';name='run_canned_place_trial.py'
prior=read(base/'canned-place-v1-20261023-deployment.json');baseline=read(base/'canned-place-v1-20261023-baseline.json')
validation=base/'canned-place-v1-20261023-cleanup-validation';validation.mkdir(exist_ok=False)
candidate=validation/'candidate'
if sha(source)!=args.sha:raise ValueError('source_hash_mismatch')
def unchanged():
    for label,root in roots.items():
        for n,digest in baseline['frozen'][label].items():
            if sha(root/n)!=digest:raise ValueError('original_file_changed:'+str(root/n))
        for n,digest in prior['hashes'].items():
            if sha(root/n)!=digest:raise ValueError('concurrent_addon_change:'+str(root/n))
    for n,digest in baseline['external'].items():
        if sha(Path(n))!=digest:raise ValueError('external_input_changed')
unchanged();shutil.copytree(roots['runtime'],candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
shutil.copyfile(source,candidate/name)
env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts'))
with (validation/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=env,stdout=f,stderr=subprocess.STDOUT,timeout=180)
output=(validation/'tests.log').read_text();print(output[-2500:],flush=True)
if proc.returncode:raise ValueError('candidate_tests_failed')
count=int(re.search(r'Ran (\d+) tests',output).group(1));unchanged()
sys.path.insert(0,str(roots['runtime']));from apple_transfer_automation import RobotLock
with RobotLock():
    unchanged()
    with tarfile.open(validation/'previous-entry.tar.gz','x:gz') as archive:
        for label,root in roots.items():archive.add(root/name,arcname=label+'/'+name)
    for root in roots.values():
        pending=root/(name+'.can-pending')
        with pending.open('xb') as f:f.write(source.read_bytes())
        pending.replace(root/name)
    for label,root in roots.items():
        for n,digest in baseline['frozen'][label].items():
            if sha(root/n)!=digest:raise ValueError('original_file_changed_after_update')
        if sha(root/name)!=args.sha:raise ValueError('deployed_entry_mismatch')
    result=dict(status='current_pose_cleanup_entry_deployed',tests=count,source_sha256=args.sha,
        previous_entry_sha256=prior['hashes'][name],all_original_code_preserved=True,
        final_placed_object_rgbd_required=True,intermediate_cleanup_requires_object_reidentification=False,
        actual_place_actions_started=False,robot_commands_sent=False)
    with (base/'canned-place-v1-20261023-entry-update.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
