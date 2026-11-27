"""Backed-up additive can-placement deployment, without robot actions."""
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
CODE=ROOTS['runtime'];PREFIX='canned-place-v1-20261023'
FILES=['canned_place_geometry.py','test_canned_place_geometry.py','run_canned_place_trial.py','CANNED_PLACE_TRIAL.txt']
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
def tests(root,path):
    env=dict(os.environ,PYTHONPATH=str(root)+':'+str(root/'scripts'))
    with path.open('x') as f:
        proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(root),'-p','test_*.py'],cwd=root,env=env,
            stdout=f,stderr=subprocess.STDOUT,timeout=180)
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
    if sha(PATCH)!=args.archive_sha:raise ValueError('archive_hash_mismatch')
    for root in ROOTS.values():
        for name in FILES:
            if (root/name).exists():raise ValueError('would_overwrite_new_file:'+str(root/name))
    names=[p.relative_to(CODE).as_posix() for p in CODE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')]
    external=[REPO/'src/handyman_rebuild_ros2/config/environments/layout_a.yaml',BASE/'active_detection_model.json',
        BASE/'handyman30-background-ft-v1/weights/best.pt']
    baseline=dict(branch=branch,archive_sha256=args.archive_sha,
        frozen={label:{name:sha(root/name) for name in names if (root/name).is_file()} for label,root in ROOTS.items()},
        external={str(p):sha(p) for p in external},robot_commands_sent=False)
    new_json(BASELINE,baseline)
    with tarfile.open(BASE/(PREFIX+'-before-addition.tar.gz'),'x:gz') as archive:
        archive.add(BASELINE,arcname=BASELINE.name)
        for label,root in ROOTS.items():
            for name in ('config/apple_transfer_auto_v1.json','inverse_table_place_plan.py','apple_transfer_auto_plan.py','apple_transfer_auto_backend.py'):
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
    sys.path.insert(0,str(CANDIDATE))
    import numpy as np
    from canned_place_geometry import make_plan,held_geometry
    measured=read(BASE/'can-destination-measured-20261023-01/result.json');ref=read(BASE/'canned-grasp-lift-live-progress-20261023.json')
    cfg=read(CANDIDATE/'config/apple_transfer_auto_v1.json');held=held_geometry(ref);replay=[]
    for row in measured['rows']:
        path=Path(row['capture']);snap=read(path/'snapshot.json');depth=np.load(path/'depth_m.npy',allow_pickle=False)
        plan=make_plan(depth,snap,measured['table'],row['plane'],ref,cfg,'recorded-placement-plan')
        bottom_gap=plan['palm_height_m']+held['center_minus_palm_m'][2]-plan['height_m']/2-plan['table_height_at_release_m']
        if abs(bottom_gap-cfg['policy']['release_gap_m'])>1e-8 or not plan['table_edge_safety']['safe']:
            raise ValueError('actual_depth_can_height_or_edge_plan_failed')
        replay.append(dict(capture=str(path),release_xy=plan['release_xy_map_m'],height=plan['palm_height_m'],
            expected_can_bottom_gap_m=bottom_gap,full_approach_distance_m=plan['planned_approach_distance_m']))
    frozen(baseline)
    result=dict(status='candidate_tests_and_actual_table_replay_passed',candidate_tests=count,recorded_table_frames=len(replay),
        actual_table_replay=replay,hashes={name:sha(CANDIDATE/name) for name in FILES},archive_sha256=args.archive_sha,robot_commands_sent=False)
    new_json(VALIDATION/'result.json',result);print(json.dumps(result),flush=True)
else:
    if REPORT.exists():raise ValueError('new_deployment_required')
    baseline=read(BASELINE);tested=read(VALIDATION/'result.json')
    if tested['status']!='candidate_tests_and_actual_table_replay_passed':raise ValueError('validated_candidate_required')
    previous=read(BASE/'canned-carry-navigation-progress-20261023.json')
    if not previous['holding_after_navigation_verified'] or previous['boot_id']!=Path('/proc/sys/kernel/random/boot_id').read_text().strip():
        raise ValueError('same_boot_held_navigation_required')
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
        result=dict(status='additive_can_placement_deployed',files=FILES,hashes=tested['hashes'],runtime_tests=count,
            candidate_tests=tested['candidate_tests'],actual_table_replay=tested['actual_table_replay'],
            original_apple_code_model_route_tolerances_unchanged=True,robot_commands_sent=False,code_copies=3,
            physical_can_placement_verified=False)
        new_json(REPORT,result);print(json.dumps(result),flush=True)
