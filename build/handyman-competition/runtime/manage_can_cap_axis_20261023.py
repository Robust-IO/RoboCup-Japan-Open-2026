"""Reviewed can-only geometry update with real-frame replay; no motion."""
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
BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');REPO=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
ROOTS=dict(runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),training=REPO/'training',module=REPO/'training/automation/apple_transfer_v1')
CODE=ROOTS['runtime'];PREFIX='can-cap-axis-v1-20261023'
CHANGED=['canned_grasp_geometry.py','canned_carry_geometry.py','held_can_appearance.py',
    'run_canned_grasp_trial.py','run_canned_carry_trial.py','run_canned_place_trial.py',
    'can_transfer_context.py','canned_place_geometry.py','OBJECT_TRANSFER.txt']
ADDED=['tracked_can_cap_geometry.py','test_tracked_can_cap_geometry.py'];FILES=CHANGED+ADDED
PATCH=BASE/(PREFIX+'-patch.tar.gz');VALIDATION=BASE/(PREFIX+'-validation');CANDIDATE=VALIDATION/'candidate'
BASELINE=BASE/(PREFIX+'-baseline.json');REPORT=BASE/(PREFIX+'-deployment.json')
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def new_json(path,value):
    with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
def frozen(baseline):
    for label,items in baseline['frozen'].items():
        for name,digest in items.items():
            if sha(ROOTS[label]/name)!=digest:raise ValueError('original_file_changed:'+label+'/'+name)
    for name,digest in baseline['external'].items():
        if sha(Path(name))!=digest:raise ValueError('external_input_changed:'+name)
def previous(baseline):
    for label,root in ROOTS.items():
        for name,digest in baseline['previous'][label].items():
            if sha(root/name)!=digest:raise ValueError('concurrent_geometry_edit:'+str(root/name))
        if any((root/name).exists() for name in ADDED):raise ValueError('new_file_already_exists')
if args.mode=='stage':
    if BASELINE.exists() or VALIDATION.exists():raise ValueError('new_validation_required')
    branch=subprocess.check_output(['git','-C',str(REPO),'branch','--show-current'],text=True).strip()
    if branch!='feat/handyman-phase5-grasping':raise ValueError('unexpected_branch')
    if sha(PATCH)!=args.archive_sha:raise ValueError('patch_hash_mismatch')
    expected=read(BASE/'can-continuous-entry-v1-20261023-deployment.json')['hashes'].copy()
    for file,report in [('canned_grasp_geometry.py','canned-grasp-robust-v2-20261023-deployment.json'),
        ('canned_carry_geometry.py','canned-carry-v1-20261023-deployment.json'),
        ('held_can_appearance.py','held-can-navigation-v1-20261023-deployment.json'),
        ('canned_place_geometry.py','canned-place-v1-20261023-deployment.json')]:
        expected[file]=read(BASE/report)['hashes'][file]
    for root in ROOTS.values():
        for name in CHANGED:
            if sha(root/name)!=expected[name]:raise ValueError('unexpected_existing_geometry:'+str(root/name))
    names=[p.relative_to(CODE).as_posix() for p in CODE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')]
    cfg=read(CODE/'config/apple_transfer_auto_v1.json');registry=read(Path(cfg['registry']))
    external=[Path(cfg['environment']),Path(cfg['semantic_map']),Path(cfg['registry']),Path(registry['weights']),BASE/'can-auto-live-20261023-01/result.json',BASE/'can-auto-live-20261023-01/grasp/result.json']
    baseline=dict(branch=branch,frozen={label:{n:sha(root/n) for n in names if n not in CHANGED and (root/n).is_file()} for label,root in ROOTS.items()},
        previous={label:{n:sha(root/n) for n in CHANGED} for label,root in ROOTS.items()},external={str(p):sha(p) for p in external})
    previous(baseline);new_json(BASELINE,baseline)
    with tarfile.open(BASE/(PREFIX+'-before-update.tar.gz'),'x:gz') as archive:
        archive.add(BASELINE,arcname=BASELINE.name)
        for label,root in ROOTS.items():
            for name in CHANGED:archive.add(root/name,arcname=label+'/'+name)
    VALIDATION.mkdir();shutil.copytree(CODE,CANDIDATE,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    with tarfile.open(PATCH) as archive:
        if set(archive.getnames())!=set(FILES):raise ValueError('unexpected_archive_files')
        for m in archive.getmembers():
            if not m.isfile() or Path(m.name).name!=m.name:raise ValueError('unsafe_member')
            with (CANDIDATE/m.name).open('wb' if m.name in CHANGED else 'xb') as f:f.write(archive.extractfile(m).read())
    for name in FILES:
        if name.endswith('.py'):ast.parse((CANDIDATE/name).read_text())
    env=dict(os.environ,PYTHONPATH=str(CANDIDATE)+':'+str(CANDIDATE/'scripts'))
    with (VALIDATION/'tests.log').open('x') as f:
        proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(CANDIDATE),'-p','test_*.py'],cwd=CANDIDATE,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=180)
    output=(VALIDATION/'tests.log').read_text();print(output[-2200:],flush=True)
    if proc.returncode:raise ValueError('tests_failed')
    count=int(re.search(r'Ran (\d+) tests',output).group(1));replays={}
    for name,script,flag in [('cap-axis','validate_can_cap_axis_20261023.py','--code'),('prior-integration','validate_can_continuous_integration_20261023.py','--candidate')]:
        with (VALIDATION/(name+'.log')).open('x') as f:
            proc=subprocess.run([sys.executable,str(BASE/script),flag,str(CANDIDATE),'--output',str(VALIDATION/name)],stdout=f,stderr=subprocess.STDOUT,timeout=180)
        if proc.returncode:
            print((VALIDATION/(name+'.log')).read_text()[-7000:],flush=True);raise ValueError('actual_replay_failed:'+name)
        replays[name]=read(VALIDATION/name/'result.json')
    frozen(baseline);previous(baseline)
    result=dict(status='candidate_tests_and_tilted_actual_replay_passed',tests=count,
        hashes={n:sha(CANDIDATE/n) for n in FILES},actual_case_frames=[t['frames'] for t in replays['cap-axis']['trials']],
        prior_integration=replays['prior-integration'],robot_commands_sent=False)
    new_json(VALIDATION/'result.json',result);print(json.dumps(result),flush=True)
else:
    if REPORT.exists():raise ValueError('new_deployment_required')
    baseline=read(BASELINE);tested=read(VALIDATION/'result.json');current=read(BASE/(PREFIX+'-current-verification/result.json'))
    if tested['status']!='candidate_tests_and_tilted_actual_replay_passed' or current['status']!='current_can_held_after_tilted_grasp_verified':
        raise ValueError('tested_code_and_current_held_audit_required')
    sys.path.insert(0,str(CODE));from apple_transfer_automation import RobotLock
    with RobotLock():
        frozen(baseline);previous(baseline)
        for n,digest in tested['hashes'].items():
            if sha(CANDIDATE/n)!=digest:raise ValueError('candidate_changed')
        for root in ROOTS.values():
            for n in FILES:
                pending=root/(n+'.axis-pending')
                with pending.open('xb') as f:f.write((CANDIDATE/n).read_bytes())
                pending.replace(root/n)
            if any(sha(root/n)!=digest for n,digest in tested['hashes'].items()):raise ValueError('copy_mismatch')
        frozen(baseline)
        result=dict(status='cap_axis_tracking_deployed_current_held_can_verified',tests=tested['tests'],files=FILES,hashes=tested['hashes'],
            original_apple_model_map_navigation_unchanged=True,first_failed_trial_preserved=True,
            current_held_audit='D:/handyman-training-runs/'+PREFIX+'-current-verification/result.json',
            robot_commands_sent=False,continuous_success_claimed=False)
        new_json(REPORT,result);print(json.dumps(result),flush=True)
