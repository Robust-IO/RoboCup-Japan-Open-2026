"""Change only the terminal-correction count, tests/docs/metadata. No motion."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import time

RUNS=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');REPO=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
ROOTS=dict(runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),training=REPO/'training',
           module=REPO/'training/automation/apple_transfer_v1')
CODE=ROOTS['runtime'];OUT=RUNS/'apple-terminal-r18b-20261022-validation-01'
REPORT=RUNS/'apple-terminal-r18b-20261022-deployment.json'
PATCH=RUNS/'apple-terminal-r18b-20261022-patch.tar.gz'
EXPECTED='29a821150755abda399b59d3dca7705d90c18582ed8da91c0511be2f80980bb9'
FILES=['grasp_terminal_correction.py','test_grasp_terminal_correction.py','APPLE_TRANSFER_AUTOMATION.txt']
CHECKPOINT='apple-terminal-r18-progress-20261022.json'
METADATA=[CHECKPOINT,'phase5-visual-test-preparation-20261022.json','phase5-restart-checkpoint-20261017.json',
          'apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json',
          'apple-outcome-policy-progress-20261018.json','apple-efficiency-r17-progress-20261022.json']
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
previous=read(RUNS/'apple-terminal-r18-20261022-deployment.json')
baseline=read(RUNS/'apple-terminal-r18-20261022-baseline.json')
if OUT.exists() or REPORT.exists():raise RuntimeError('new_output_required')
if sha(PATCH)!=EXPECTED:raise RuntimeError('patch_hash_mismatch')
for root in ROOTS.values():
    for path in [root/'AGENTS.md',*(p/'AGENTS.md' for p in root.parents)]:
        if path.exists():print(str(path)+'\n'+path.read_text());raise RuntimeError('read_instructions_first')
    for name in FILES:
        if sha(root/name)!=previous['hashes'][name]:raise RuntimeError('concurrent_code_change:'+str(root/name))
OUT.mkdir();candidate=OUT/'candidate'
shutil.copytree(CODE,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git'))
with tarfile.open(PATCH) as archive:
    if set(archive.getnames())!=set(FILES):raise RuntimeError('unexpected_patch_files')
    for member in archive.getmembers():
        if not member.isfile() or Path(member.name).name!=member.name:raise RuntimeError('unsafe_patch_member')
        with (candidate/member.name).open('wb') as stream:stream.write(archive.extractfile(member).read())
old=(CODE/'grasp_terminal_correction.py').read_text()
if old.replace('MAX_CORRECTIONS = 2','MAX_CORRECTIONS = 5',1)!=(candidate/'grasp_terminal_correction.py').read_text():
    raise RuntimeError('runtime_change_exceeds_requested_count')
env=os.environ.copy();env['PYTHONPATH']=str(candidate)+':'+str(candidate/'scripts')
with (OUT/'candidate-tests.log').open('x') as log:
    check=subprocess.run([sys.executable,'-m','unittest','test_grasp_terminal_correction'],cwd=candidate,
        env=env,stdout=log,stderr=subprocess.STDOUT,timeout=60)
if check.returncode:raise RuntimeError('candidate_tests_failed')
sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json
with RobotLock():
    roots=dict(runs=RUNS,**ROOTS)
    for name in METADATA:
        if len({sha(root/name) for root in roots.values()})!=1:raise RuntimeError('metadata_disagrees')
    frozen={label:{name:sha(root/name) for name in baseline['frozen'][label]} for label,root in ROOTS.items()}
    with tarfile.open(RUNS/'apple-before-terminal-r18b-20261022.tar.gz','x:gz') as archive:
        for label,root in ROOTS.items():
            for name in FILES:archive.add(root/name,arcname=label+'/'+name)
        for label,root in roots.items():
            for name in METADATA:archive.add(root/name,arcname='metadata/'+label+'/'+name)
    for root in ROOTS.values():
        for name in FILES:
            if sha(root/name)!=previous['hashes'][name]:raise RuntimeError('source_changed_before_copy')
            temp=root/(name+'.r18b-deploy-pending');shutil.copyfile(candidate/name,temp);temp.replace(root/name)
    hashes={name:sha(candidate/name) for name in FILES}
    if any(sha(root/name)!=digest for root in ROOTS.values() for name,digest in hashes.items()):raise RuntimeError('deployed_copies_mismatch')
    if any(sha(ROOTS[label]/name)!=digest for label,files in frozen.items() for name,digest in files.items()):
        raise RuntimeError('navigation_or_other_frozen_code_changed')
    env['PYTHONPATH']=str(CODE)+':'+str(CODE/'scripts')
    with (OUT/'runtime-tests.log').open('x') as log:
        test=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(CODE),'-p','test_*.py'],cwd=CODE,
            env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180)
    text=(OUT/'runtime-tests.log').read_text();print(text[-2000:],flush=True)
    if test.returncode:raise RuntimeError('runtime_tests_failed_no_motion_started')
    count=int(re.search(r'Ran (\d+) tests',text).group(1))
    updates={}
    for name in METADATA:
        value=read(RUNS/name)
        value.update(current_code_revision='terminal-correction-r18b')
        if 'next' in value:value['next']=value['next'].replace('terminal-correction-r18 is deployed','terminal-correction-r18b is deployed').replace('up to2 NEW','up to5 NEW')
        if name==CHECKPOINT:
            value.update(revision='terminal-correction-r18b',maximum_terminal_corrections=5,
                saved_ns=time.time_ns(),tests=count,deployment=str(REPORT),
                limit_update_validation=str(OUT/'runtime-tests.log'),robot_commands_sent=False)
        if name=='apple-transfer-automation-progress-20261017.json':
            value.update(implementation_revision='20261022-terminal-correction-r18b-limit5',
                latest_patch='D:/handyman-training-runs/apple-terminal-r18b-20261022-patch.tar.gz',latest_patch_sha256=EXPECTED)
        updates[name]=value
    for root in roots.values():
        for name,value in updates.items():write_json(root/name,value)
    metadata_hashes={name:sha(RUNS/name) for name in updates}
    if any(sha(root/name)!=digest for root in roots.values() for name,digest in metadata_hashes.items()):raise RuntimeError('metadata_sync_mismatch')
    result=dict(status='deployed_tests_passed',revision='terminal-correction-r18b',maximum_terminal_corrections=5,
        tests=count,hashes=hashes,checkpoint_hashes=metadata_hashes,archive_sha256=EXPECTED,
        runtime_change='MAX_CORRECTIONS:2->5 only',navigation_and_all_other_runtime_logic_unchanged=True,
        robot_commands_sent=False,new_live_trial=False)
    with REPORT.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2),flush=True)
