"""Audited update to the generic observation addon only; no action entrypoints."""
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
CODE=ROOTS['runtime'];PREFIX='canned-carry-v1-20261023'
FILES=['canned_grasp_runtime.py','canned_carry_geometry.py','run_canned_carry_trial.py','test_canned_carry_geometry.py','CANNED_CARRY_TRIAL.txt']
PATCH=BASE/(PREFIX+'-patch-03.tar.gz');EXPECTED='d8b7f33211b5c88e7c6e70f4220fa245a1321ea261e52fc8eb7f2fa2fc73c5e7'
VALIDATION=BASE/(PREFIX+'-validation-03');CANDIDATE=VALIDATION/'candidate'
BASELINE=BASE/(PREFIX+'-baseline-03.json');REPORT=BASE/(PREFIX+'-deployment.json')
read=lambda p:json.loads(p.read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def new_json(p,value):
    with p.open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False)
def verify_frozen(baseline):
    for label,root in ROOTS.items():
        for name,digest in baseline['frozen'][label].items():
            if sha(root/name)!=digest:raise ValueError('existing_nonaddon_file_changed:'+label+'/'+name)
def verify_previous(baseline):
    for label,root in ROOTS.items():
        for name,digest in baseline['previous'][label].items():
            if digest is None:
                if (root/name).exists():raise ValueError('unexpected_new_file:'+label+'/'+name)
            elif sha(root/name)!=digest:raise ValueError('concurrent_addon_edit:'+label+'/'+name)
def tests(root,log):
    env=dict(os.environ,PYTHONPATH=str(root)+':'+str(root/'scripts'))
    with log.open('x') as stream:
        result=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(root),'-p','test_*.py'],
            cwd=root,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=180)
    output=log.read_text();print(output[-2500:],flush=True)
    if result.returncode:raise ValueError('tests_failed:'+str(log))
    return int(re.search(r'Ran (\d+) tests',output).group(1))
if args.mode=='stage':
    if BASELINE.exists() or VALIDATION.exists():raise ValueError('new_validation_required')
    for root in ROOTS.values():
        for p in {root/'AGENTS.md',root/'config/AGENTS.md',*(p/'AGENTS.md' for p in root.parents)}:
            if p.exists():
                print(str(p)+'\n'+p.read_text(),flush=True);raise ValueError('read_applicable_instructions_first')
    branch=subprocess.check_output(['git','-C',str(REPO),'branch','--show-current'],text=True).strip()
    if branch!='feat/handyman-phase5-grasping':raise ValueError('unexpected_branch')
    prior=read(BASE/'canned-grasp-robust-v2-20261023-deployment.json')
    for root in ROOTS.values():
        for name,digest in prior['hashes'].items():
            if sha(root/name)!=digest:raise ValueError('prior_addon_changed:'+str(root/name))
        for name in set(FILES)-set(prior['hashes']):
            if (root/name).exists():raise ValueError('would_overwrite_new_file:'+str(root/name))
    if sha(PATCH)!=EXPECTED:raise ValueError('archive_hash_mismatch')
    existing=[p.relative_to(CODE).as_posix() for p in CODE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')]
    baseline=dict(branch=branch,
        frozen={label:{name:sha(root/name) for name in existing if name not in FILES and (root/name).is_file()} for label,root in ROOTS.items()},
        previous={label:{name:sha(root/name) if (root/name).is_file() else None for name in FILES} for label,root in ROOTS.items()},
        robot_commands_sent=False)
    new_json(BASELINE,baseline)
    with tarfile.open(BASE/(PREFIX+'-before-addon-update-03.tar.gz'),'x:gz') as archive:
        for label,root in ROOTS.items():
            for name,digest in baseline['previous'][label].items():
                if digest is not None:archive.add(root/name,arcname=label+'/'+name)
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
        proc=subprocess.run([str(REPO/'.pixi/envs/default/bin/python'),str(BASE/'validate_canned_lift_replay_20261023.py'),
            '--candidate',str(CANDIDATE),'--output',str(VALIDATION/'recorded-checks')],stdout=stream,stderr=subprocess.STDOUT,timeout=180)
    if proc.returncode:
        print((VALIDATION/'recorded-checks.log').read_text()[-6000:],flush=True);raise ValueError('recorded_checks_failed')
    replay=read(VALIDATION/'recorded-checks/result.json')
    if replay.get('can_recorded_lift_follow_verified') is not True:raise ValueError('actual_rgbd_replay_required')
    sys.path.insert(0,str(CANDIDATE))
    from generic_transfer_template import run_offline,OfflineBackend
    from manipulation_profiles import transfer_config
    smoke={name:run_offline(transfer_config(name),'offline-'+name,VALIDATION/('smoke-'+name),OfflineBackend()) for name in ('apple','rubiks_cube','canned_juice')}
    if any(v['physical_transfer_verified'] or len(v['confirmed_stages'])!=17 for v in smoke.values()):raise ValueError('offline_flow_incorrectly_claims_physical_success')
    verify_frozen(baseline);verify_previous(baseline)
    result=dict(status='candidate_tests_replay_and_offline_flows_passed',candidate_tests=count,replay=replay,
        hashes={name:sha(CANDIDATE/name) for name in FILES},archive_sha256=EXPECTED,
        original_apple_runtime_preserved=True,robot_commands_sent=False)
    new_json(VALIDATION/'result.json',result);print(json.dumps({k:result[k] for k in ('status','candidate_tests','hashes','archive_sha256','robot_commands_sent')}),flush=True)
else:
    if REPORT.exists():raise ValueError('new_deployment_required')
    baseline=read(BASELINE);tested=read(VALIDATION/'result.json')
    if tested['status']!='candidate_tests_replay_and_offline_flows_passed':raise ValueError('validated_candidate_required')
    sys.path.insert(0,str(CODE))
    from apple_transfer_automation import RobotLock,write_json
    with RobotLock():
        verify_frozen(baseline);verify_previous(baseline)
        for name,digest in tested['hashes'].items():
            if sha(CANDIDATE/name)!=digest:raise ValueError('candidate_changed')
        for root in ROOTS.values():
            for name in FILES:
                dest=root/name;dest.parent.mkdir(parents=True,exist_ok=True)
                pending=dest.with_name(dest.name+'.canned-pending')
                with pending.open('xb') as stream:stream.write((CANDIDATE/name).read_bytes())
                pending.replace(dest)
        verify_frozen(baseline)
        for root in ROOTS.values():
            if any(sha(root/name)!=digest for name,digest in tested['hashes'].items()):raise ValueError('deployed_copy_mismatch')
        count=tests(CODE,VALIDATION/'runtime-tests.log');verify_frozen(baseline)
        checkpoint='canned-carry-adapter-progress-20261023.json'
        if any((root/checkpoint).exists() for root in [BASE,*ROOTS.values()]):raise ValueError('new_progress_required')
        held=read(BASE/'can-before-carry-20261023-05/result.json')
        if held.get('status')!='current_extended_can_held' or held.get('robot_commands_sent') is not False:raise ValueError('current_readonly_held_verification_required')
        progress=dict(status='can_retraction_navigation_entry_deployed_not_yet_executed',saved_ns=time.time_ns(),
            branch=baseline['branch'],scope='Can-only retraction and optional original navigation. No grasp, closure, release, placement or Task_finished.',
            runtime_tests=count,candidate_tests=tested['candidate_tests'],deployment=str(REPORT),files=FILES,
            live_entry='run_canned_carry_trial.py --run --navigate-destination --output NEW_D_DIRECTORY',
            grasp_capture_default_processing_reserve_ns=200000000,
            carry_without_detector_processing_reserve_ns=0,final_sample_age_limit_s=.4,
            navigation_tolerances=[.06,.08,.06],route_and_speed_policy_unchanged=True,
            last_readonly_scene='D:/handyman-training-runs/can-before-carry-20261023-05/result.json',
            previous_grasp='D:/handyman-training-runs/canned-grasp-lift-live-progress-20261023.json',
            can_carry_live_verified=False,can_placement_verified=False,robot_commands_sent=False,
            next='Run the explicit new carry entry from currently held extended arm. Observe current can depth, preserve grip during folding/lowering, verify actual tucked held can, then follow the ORIGINAL bedroom-to-living_room route to square_low_table staging and verify actual held can again. Never re-run grasp or release a held hand. On any action failure do not replay automatically; audit physical state.')
        for root in [BASE,*ROOTS.values()]:write_json(root/checkpoint,progress)
        result=dict(status='can_observation_addon_update_verified',files=FILES,hashes=tested['hashes'],
            runtime_tests=count,candidate_tests=tested['candidate_tests'],original_apple_runtime_preserved=True,
            source_copies=3,checkpoint_copies=4,checkpoint='D:/handyman-training-runs/'+checkpoint,
            checkpoint_sha256=sha(BASE/checkpoint),robot_commands_sent=False,can_live_mover_complete=False)
        new_json(REPORT,result);print(json.dumps(result),flush=True)
