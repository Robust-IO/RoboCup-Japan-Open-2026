"""Additive, backed-up deployment; never runs robot actions."""
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

p=argparse.ArgumentParser();p.add_argument('mode',choices=['stage','deploy']);p.add_argument('--archive-sha',required=True);args=p.parse_args()
BASE=Path('/__HANDYMAN_PACKAGE__/runtime');REPO=Path('/__HANDYMAN_PACKAGE__/project')
ROOTS=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),training=REPO/'training',module=REPO/'training/automation/apple_transfer_v1')
CODE=ROOTS['runtime'];PREFIX='held-can-navigation-v1-20261023'
FILES=['held_can_appearance.py','test_held_can_appearance.py','run_tucked_can_navigation.py','TUCKED_CAN_NAVIGATION.txt']
PATCH=BASE/(PREFIX+'-patch.tar.gz');VALIDATION=BASE/(PREFIX+'-validation');CANDIDATE=VALIDATION/'candidate'
BASELINE=BASE/(PREFIX+'-baseline.json');REPORT=BASE/(PREFIX+'-deployment.json')
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def new_json(path,value):
    with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
def frozen(baseline):
    for label,items in baseline['frozen'].items():
        for name,digest in items.items():
            if sha(ROOTS[label]/name)!=digest:raise ValueError('existing_file_changed:'+label+'/'+name)
    for name,digest in baseline['external'].items():
        if sha(Path(name))!=digest:raise ValueError('external_input_changed:'+name)
def tests(root,path):
    env=dict(os.environ,PYTHONPATH=str(root)+':'+str(root/'scripts'))
    with path.open('x') as f:
        proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(root),'-p','test_*.py'],
            cwd=root,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=180)
    output=path.read_text();print(output[-2500:],flush=True)
    if proc.returncode:raise ValueError('tests_failed:'+str(path))
    return int(re.search(r'Ran (\d+) tests',output).group(1))
if args.mode=='stage':
    if BASELINE.exists() or VALIDATION.exists():raise ValueError('new_validation_required')
    for root in ROOTS.values():
        for path in {root/'AGENTS.md',*(p/'AGENTS.md' for p in root.parents)}:
            if path.exists():raise ValueError('read_applicable_instructions_first:'+str(path))
    branch=subprocess.check_output(['git','-C',str(REPO),'branch','--show-current'],text=True).strip()
    if branch!='feat/handyman-phase5-grasping':raise ValueError('unexpected_branch')
    for root in ROOTS.values():
        for name in FILES:
            if (root/name).exists():raise ValueError('would_overwrite_new_file:'+str(root/name))
    if sha(PATCH)!=args.archive_sha:raise ValueError('archive_hash_mismatch')
    prior=read(BASE/'canned-carry-v1-20261023-deployment.json')
    for root in ROOTS.values():
        for name,digest in prior['hashes'].items():
            if sha(root/name)!=digest:raise ValueError('previous_carry_code_changed:'+str(root/name))
    names=[p.relative_to(CODE).as_posix() for p in CODE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')]
    external=[REPO/'src/handyman_rebuild_ros2/config/environments/layout_a.yaml',
        BASE/'active_detection_model.json',BASE/'handyman30-background-ft-v1/weights/best.pt']
    baseline=dict(branch=branch,archive_sha256=args.archive_sha,
        frozen={label:{name:sha(root/name) for name in names if (root/name).is_file()} for label,root in ROOTS.items()},
        external={str(p):sha(p) for p in external},robot_commands_sent=False)
    new_json(BASELINE,baseline)
    with tarfile.open(BASE/(PREFIX+'-before-addition.tar.gz'),'x:gz') as archive:
        archive.add(BASELINE,arcname=BASELINE.name)
        for label,root in ROOTS.items():
            for name in ('config/apple_transfer_auto_v1.json','canned_carry_geometry.py','canned_grasp_runtime.py','apple_transfer_auto_backend.py'):
                if (root/name).is_file():archive.add(root/name,arcname=label+'/'+name)
    VALIDATION.mkdir();shutil.copytree(CODE,CANDIDATE,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    with tarfile.open(PATCH) as archive:
        if set(archive.getnames())!=set(FILES):raise ValueError('unexpected_archive_files')
        for member in archive.getmembers():
            if not member.isfile() or Path(member.name).name!=member.name:raise ValueError('unsafe_member')
            with (CANDIDATE/member.name).open('xb') as f:f.write(archive.extractfile(member).read())
    for name in FILES:
        if name.endswith('.py'):ast.parse((CANDIDATE/name).read_text())
    count=tests(CANDIDATE,VALIDATION/'candidate-tests.log')
    with (VALIDATION/'recorded-checks.log').open('x') as f:
        proc=subprocess.run([sys.executable,str(BASE/'validate_held_can_appearance_20261023.py'),
            '--candidate',str(CANDIDATE),'--output',str(VALIDATION/'recorded-checks')],stdout=f,stderr=subprocess.STDOUT,timeout=90)
    if proc.returncode:
        print((VALIDATION/'recorded-checks.log').read_text()[-5000:],flush=True);raise ValueError('actual_recorded_replay_failed')
    replay=read(VALIDATION/'recorded-checks/result.json');frozen(baseline)
    result=dict(status='candidate_tests_and_actual_recorded_replay_passed',candidate_tests=count,recorded_frames=replay['frames'],
        hashes={name:sha(CANDIDATE/name) for name in FILES},archive_sha256=args.archive_sha,robot_commands_sent=False)
    new_json(VALIDATION/'result.json',result);print(json.dumps(result),flush=True)
else:
    if REPORT.exists():raise ValueError('new_deployment_required')
    baseline=read(BASELINE);tested=read(VALIDATION/'result.json');current=read(BASE/(PREFIX+'-current-observation/result.json'))
    if tested['status']!='candidate_tests_and_actual_recorded_replay_passed':raise ValueError('validated_candidate_required')
    if current.get('status')!='current_tucked_can_appearance_verified' or current.get('robot_commands_sent') is not False:
        raise ValueError('successful_current_readonly_observation_required')
    if read(BASE/'canned-carry-retracted-progress-20261023.json')['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
        raise ValueError('same_boot_required')
    sys.path.insert(0,str(CODE));from apple_transfer_automation import RobotLock
    with RobotLock():
        frozen(baseline)
        for name,digest in tested['hashes'].items():
            if sha(CANDIDATE/name)!=digest:raise ValueError('candidate_changed')
        for root in ROOTS.values():
            for name in FILES:
                with (root/name).open('xb') as f:f.write((CANDIDATE/name).read_bytes())
        for root in ROOTS.values():
            if any(sha(root/name)!=digest for name,digest in tested['hashes'].items()):raise ValueError('copy_mismatch')
        count=tests(CODE,VALIDATION/'runtime-tests.log');frozen(baseline)
        result=dict(status='additive_held_can_navigation_deployed',files=FILES,hashes=tested['hashes'],
            runtime_tests=count,candidate_tests=tested['candidate_tests'],recorded_frames=tested['recorded_frames'],
            actual_current_readonly_verification=str(BASE/(PREFIX+'-current-observation/result.json')),
            old_code_model_route_navigation_parameters_unchanged=True,robot_commands_sent=False,
            code_copies=3,automatic_navigation_started=False)
        new_json(REPORT,result);print(json.dumps(result),flush=True)
