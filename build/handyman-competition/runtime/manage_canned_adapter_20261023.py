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
CODE=ROOTS['runtime'];PREFIX='generic-canned-juice-v2-20261023'
FILES=['config/manipulation_objects_v1.json','manipulation_profiles.py','object_surface_geometry.py',
       'upright_cylinder_geometry.py','generic_transfer_template.py','inspect_manipulation_capture.py',
       'test_generic_transfer_template.py','test_upright_cylinder_geometry.py','GENERIC_TRANSFER_TEMPLATE.txt']
PATCH=BASE/(PREFIX+'-patch.tar.gz');EXPECTED='793691f9a377e23642cbd4fa7b379fbc2c7b7ac71f7d670a9e565f1411edb428'
VALIDATION=BASE/(PREFIX+'-validation-01');CANDIDATE=VALIDATION/'candidate'
BASELINE=BASE/(PREFIX+'-baseline-01.json');REPORT=BASE/(PREFIX+'-deployment.json')
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
    prior=read(BASE/'generic-transfer-v1-20261023-deployment.json')
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
    with tarfile.open(BASE/(PREFIX+'-before-addon-update.tar.gz'),'x:gz') as archive:
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
        proc=subprocess.run([str(REPO/'.pixi/envs/default/bin/python'),str(BASE/'validate_canned_models_20261023.py'),
            '--candidate',str(CANDIDATE),'--output',str(VALIDATION/'recorded-checks')],stdout=stream,stderr=subprocess.STDOUT,timeout=180)
    if proc.returncode:
        print((VALIDATION/'recorded-checks.log').read_text()[-6000:],flush=True);raise ValueError('recorded_checks_failed')
    replay=read(VALIDATION/'recorded-checks/result.json')
    sys.path.insert(0,str(CANDIDATE))
    from generic_transfer_template import run_offline,OfflineBackend
    from manipulation_profiles import transfer_config
    smoke={name:run_offline(transfer_config(name),'offline-'+name,VALIDATION/('smoke-'+name),OfflineBackend()) for name in ('apple','rubiks_cube','canned_juice')}
    if any(v['physical_transfer_verified'] or len(v['confirmed_stages'])!=17 for v in smoke.values()):raise ValueError('offline_flow_incorrectly_claims_physical_success')
    verify_frozen(baseline);verify_previous(baseline)
    result=dict(status='candidate_tests_replay_and_offline_flows_passed',candidate_tests=count,replay=replay,
        hashes={name:sha(CANDIDATE/name) for name in FILES},archive_sha256=EXPECTED,
        original_apple_runtime_preserved=True,robot_commands_sent=False)
    new_json(VALIDATION/'result.json',result);print(json.dumps(result),flush=True)
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
        checkpoint='generic-canned-juice-progress-20261023.json'
        if any((root/checkpoint).exists() for root in [BASE,*ROOTS.values()]):raise ValueError('new_progress_required')
        progress=dict(status='can_observation_adapter_deployed_live_validation_pending',saved_ns=time.time_ns(),
            branch=baseline['branch'],supported_observation_profiles=['apple','rubiks_cube','canned_juice'],
            first_test_object='canned_juice',reason='User confirmed LayoutA has no rubiks_cube.',
            generic_live_motion_enabled=False,can_fresh_rgbd_validated=False,can_grasp_lift_carry_placement_verified=False,
            apple_legacy_entry_unchanged=True,apple_previous_full_live_seconds=420.144510206,
            runtime_tests=count,candidate_tests=tested['candidate_tests'],recorded_replay=tested['replay'],
            deployment=str(REPORT),files=FILES,robot_commands_sent=False,model_or_navigation_changed=False,
            communication='Foreground WSL client holds original 9090/50001 bridges. No Unity connection received at last check; inspect Unity endpoint/Console before restarting. Boot and timestamps must be rechecked.',
            bridge_supervisor='D:/handyman-training-runs/hold_canned_bridges_20261023.py',
            next='Receive fresh head/hand RGBD and joints, visually confirm canned_juice and validate measured cylinder geometry before motion. No old coordinates, scene poses or offline successes are live transfer evidence.')
        for root in [BASE,*ROOTS.values()]:write_json(root/checkpoint,progress)
        result=dict(status='can_observation_addon_update_verified',files=FILES,hashes=tested['hashes'],
            runtime_tests=count,candidate_tests=tested['candidate_tests'],original_apple_runtime_preserved=True,
            source_copies=3,checkpoint_copies=4,checkpoint='D:/handyman-training-runs/'+checkpoint,
            checkpoint_sha256=sha(BASE/checkpoint),robot_commands_sent=False,can_live_mover_complete=False)
        new_json(REPORT,result);print(json.dumps(result),flush=True)
