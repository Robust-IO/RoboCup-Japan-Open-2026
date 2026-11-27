"""Additive, checksum-audited deployment. Never invokes ROS/action entrypoints."""
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
import time

parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['stage','deploy']);args=parser.parse_args()
BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');REPO=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
ROOTS=dict(runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),training=REPO/'training',module=REPO/'training/automation/apple_transfer_v1')
CODE=ROOTS['runtime'];PREFIX='canned-grasp-v1-20261023'
FILES=['canned_grasp_geometry.py','canned_grasp_runtime.py','run_canned_grasp_trial.py','test_canned_grasp_geometry.py','CANNED_GRASP_TRIAL.txt']
PATCH=BASE/(PREFIX+'-patch.tar.gz');EXPECTED='c056d77e297316945e9588eab989e8e93fdfafba4c5e565108b5b7ec7f204a16'
VALIDATION=BASE/(PREFIX+'-validation-01');CANDIDATE=VALIDATION/'candidate'
BASELINE=BASE/(PREFIX+'-baseline-01.json');REPORT=BASE/(PREFIX+'-deployment.json')
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def new_json(p,value):
    with p.open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False)
def old_files():return [p.relative_to(CODE).as_posix() for p in CODE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')]
def frozen_unchanged(baseline):
    for label,root in ROOTS.items():
        for name,digest in baseline['frozen'][label].items():
            if sha(root/name)!=digest:raise ValueError('existing_file_changed:'+label+'/'+name)
def tests(root,log):
    env=dict(os.environ,PYTHONPATH=str(root)+':'+str(root/'scripts'))
    with log.open('x') as stream:
        result=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(root),'-p','test_*.py'],cwd=root,
                              env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=180)
    output=log.read_text();print(output[-2500:],flush=True)
    if result.returncode:raise ValueError('tests_failed:'+str(log))
    return int(re.search(r'Ran (\d+) tests',output).group(1))
if args.mode=='stage':
    if BASELINE.exists() or VALIDATION.exists():raise ValueError('new_validation_required')
    for root in ROOTS.values():
        for path in [root/'AGENTS.md',root/'config/AGENTS.md',*(p/'AGENTS.md' for p in root.parents)]:
            if path.exists():
                print(str(path)+'\n'+path.read_text(),flush=True);raise ValueError('read_applicable_instructions_first')
    branch=subprocess.check_output(['git','-C',str(REPO),'branch','--show-current'],text=True).strip()
    if branch!='feat/handyman-phase5-grasping':raise ValueError('unexpected_branch')
    for root in ROOTS.values():
        if any((root/name).exists() for name in FILES):raise ValueError('new_file_would_overwrite_user_work')
    if sha(PATCH)!=EXPECTED:raise ValueError('archive_hash_mismatch')
    existing=old_files()
    baseline=dict(branch=branch,existing_runtime_file_count=len(existing),
        frozen={label:{name:sha(root/name) for name in existing if (root/name).is_file()} for label,root in ROOTS.items()},
        robot_commands_sent=False)
    new_json(BASELINE,baseline)
    with tarfile.open(BASE/'runtime-before-canned-grasp-v1-20261023.tar.gz','x:gz') as archive:
        for name in existing:archive.add(CODE/name,arcname=name)
    VALIDATION.mkdir();shutil.copytree(CODE,CANDIDATE,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    with tarfile.open(PATCH) as archive:
        if set(archive.getnames())!=set(FILES):raise ValueError('unexpected_archive_files')
        for member in archive.getmembers():
            if not member.isfile() or Path(member.name).is_absolute() or '..' in Path(member.name).parts:raise ValueError('unsafe_archive_member')
            dest=CANDIDATE/member.name;dest.parent.mkdir(parents=True,exist_ok=True)
            with dest.open('wb') as stream:stream.write(archive.extractfile(member).read())
    for name in FILES:
        if name.endswith('.py'):ast.parse((CANDIDATE/name).read_text(),filename=name)
    count=tests(CANDIDATE,VALIDATION/'candidate-tests.log')
    with (VALIDATION/'recorded-checks.log').open('x') as stream:
        proc=subprocess.run([str(REPO/'.pixi/envs/default/bin/python'),str(BASE/'validate_canned_depth_replay_20261023.py'),
                             '--candidate',str(CANDIDATE),'--output',str(VALIDATION/'recorded-checks')],
                            stdout=stream,stderr=subprocess.STDOUT,timeout=180)
    if proc.returncode:
        print((VALIDATION/'recorded-checks.log').read_text()[-6000:],flush=True);raise ValueError('recorded_checks_failed')
    replay=read(VALIDATION/'recorded-checks/result.json')
    sys.path.insert(0,str(CANDIDATE))
    from canned_grasp_geometry import measure_near
    import numpy as np
    near_checks=[]
    for row in replay['rows']:
        cap=Path(replay['source_run'])/'cameras'/row['frame'];geometry=row['geometry']
        measured=measure_near(np.load(cap/'depth_m.npy',allow_pickle=False),read(cap/'snapshot.json'),geometry['center_m'],geometry['footprint_radius_m'],geometry['height_m'])
        if np.linalg.norm(np.asarray(measured['center_m'])-geometry['center_m'])>.005:raise ValueError('new_near_can_fit_disagrees')
        near_checks.append(measured)
    new_json(VALIDATION/'near-real-rgbd-checks.json',near_checks)
    sys.path.insert(0,str(CANDIDATE))
    from generic_transfer_template import run_offline,OfflineBackend
    from manipulation_profiles import transfer_config
    smoke={}
    for name in ('apple','rubiks_cube','canned_juice'):
        smoke[name]=run_offline(transfer_config(name),'offline-'+name,VALIDATION/('smoke-'+name),OfflineBackend())
    if any(v['physical_transfer_verified'] or len(v['confirmed_stages'])!=17 for v in smoke.values()):raise ValueError('offline_physical_success_or_incomplete_flow')
    frozen_unchanged(baseline)
    result=dict(status='candidate_tests_replay_and_offline_flows_passed',candidate_tests=count,replay=replay,
                hashes={name:sha(CANDIDATE/name) for name in FILES},archive_sha256=EXPECTED,
                all_existing_apple_runtime_files_unchanged=True,robot_commands_sent=False)
    new_json(VALIDATION/'result.json',result);print(json.dumps(result),flush=True)
else:
    if REPORT.exists():raise ValueError('new_deployment_required')
    baseline=read(BASELINE);tested=read(VALIDATION/'result.json')
    if tested['status']!='candidate_tests_replay_and_offline_flows_passed':raise ValueError('validated_candidate_required')
    sys.path.insert(0,str(CODE))
    from apple_transfer_automation import RobotLock,write_json
    with RobotLock():
        frozen_unchanged(baseline)
        for root in ROOTS.values():
            if any((root/name).exists() for name in FILES):raise ValueError('concurrent_new_source_file')
        for name,digest in tested['hashes'].items():
            if sha(CANDIDATE/name)!=digest:raise ValueError('candidate_changed_since_tests')
        for root in ROOTS.values():
            for name in FILES:
                dest=root/name;dest.parent.mkdir(parents=True,exist_ok=True)
                pending=dest.with_name(dest.name+'.generic-pending');shutil.copyfile(CANDIDATE/name,pending);pending.replace(dest)
        frozen_unchanged(baseline)
        for root in ROOTS.values():
            if any(sha(root/name)!=digest for name,digest in tested['hashes'].items()):raise ValueError('deployed_copy_mismatch')
        count=tests(CODE,VALIDATION/'runtime-tests.log')
        frozen_unchanged(baseline)
        checkpoint='canned-grasp-adapter-progress-20261023.json'
        if any((root/checkpoint).exists() for root in [BASE,*ROOTS.values()]):raise ValueError('new_progress_required')
        observation=read(BASE/'canned-grasp-v1-20261023-observe-01/result.json')
        if observation.get('status')!='new_can_observation_verified' or observation.get('robot_commands_sent') is not False:raise ValueError('new_readonly_observation_required')
        progress=dict(status='explicit_can_grasp_trial_deployed_not_executed',saved_ns=time.time_ns(),
            branch=baseline['branch'],scope='Can-only fixed-source explicit grasp/lift experiment, not a complete generic mover.',
            new_live_entry='run_canned_grasp_trial.py --run --simulation-ignore-collision --output NEW_D_DIRECTORY',
            generic_full_transfer_enabled=False,can_grasp_lift_carry_placement_verified=False,
            new_model_observation=str(BASE/'canned-grasp-v1-20261023-observe-01/result.json'),
            maximum_measured_corrections=5,maximum_close_commands=1,
            apple_legacy_entry_unchanged=True,model_or_navigation_changed=False,
            runtime_tests=count,candidate_tests=tested['candidate_tests'],recorded_replay=tested['replay'],
            deployment=str(REPORT),files=FILES,robot_commands_sent=False,
            next='Run the new explicit can trial from the current empty tucked bedroom table observation pose. The entry acquires NEW model detections/RGBD/TF, prepares the horizontal arm, makes one full approach plus at most five measured corrections, closes once, lifts only on stable contact and verifies actual can/palm depth following. On failure do not auto-replay or release; audit actual state. No automatic navigation, placement or Task_finished.')
        for root in [BASE,*ROOTS.values()]:write_json(root/checkpoint,progress)
        result=dict(status='additive_deployment_verified',files=FILES,hashes=tested['hashes'],runtime_tests=count,
            candidate_tests=tested['candidate_tests'],existing_runtime_files_preserved=baseline['existing_runtime_file_count'],
            all_existing_apple_runtime_files_unchanged=True,source_copies=3,checkpoint_copies=4,
            checkpoint='D:/handyman-training-runs/'+checkpoint,checkpoint_sha256=sha(BASE/checkpoint),
            robot_commands_sent=False,can_full_transfer_complete=False)
        new_json(REPORT,result);print(json.dumps(result),flush=True)
