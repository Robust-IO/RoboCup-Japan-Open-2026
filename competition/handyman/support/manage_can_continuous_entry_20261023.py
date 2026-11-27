"""Deploy only the reviewed can entry adapters, preserving the apple baseline."""
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

p=argparse.ArgumentParser();p.add_argument('mode',choices=['stage','deploy']);p.add_argument('--archive-sha',required=True);args=p.parse_args()
BASE=Path('/__HANDYMAN_PACKAGE__/runtime');REPO=Path('/__HANDYMAN_PACKAGE__/project')
ROOTS=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),training=REPO/'training',module=REPO/'training/automation/apple_transfer_v1')
CODE=ROOTS['runtime'];PREFIX='can-continuous-entry-v1-20261023'
CHANGED=['run_canned_grasp_trial.py','run_canned_carry_trial.py','run_canned_place_trial.py']
ADDED=['can_transfer_context.py','can_transfer_automation.py','run_object_transfer.py','test_can_transfer_automation.py','run_object_transfer.sh','OBJECT_TRANSFER.txt']
FILES=CHANGED+ADDED;PATCH=BASE/(PREFIX+'-patch.tar.gz');VALIDATION=BASE/(PREFIX+'-validation');CANDIDATE=VALIDATION/'candidate'
BASELINE=BASE/(PREFIX+'-baseline.json');REPORT=BASE/(PREFIX+'-deployment.json')
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def new_json(path,value):
    with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
def frozen(baseline):
    for label,items in baseline['frozen'].items():
        for name,digest in items.items():
            if sha(ROOTS[label]/name)!=digest:raise ValueError('original_file_changed:'+label+'/'+name)
    for name,digest in baseline['external'].items():
        if sha(Path(name))!=digest:raise ValueError('model_map_or_registry_changed:'+name)
def previous(baseline):
    for label,root in ROOTS.items():
        for name,digest in baseline['previous'][label].items():
            if sha(root/name)!=digest:raise ValueError('concurrent_adapter_edit:'+str(root/name))
        for name in ADDED:
            if (root/name).exists():raise ValueError('unexpected_new_file:'+str(root/name))
def tests(root,path):
    env=dict(os.environ,PYTHONPATH=str(root)+':'+str(root/'scripts'))
    with path.open('x') as f:
        proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(root),'-p','test_*.py'],cwd=root,env=env,
            stdout=f,stderr=subprocess.STDOUT,timeout=180)
    output=path.read_text();print(output[-2500:],flush=True)
    if proc.returncode:raise ValueError('candidate_tests_failed:'+str(path))
    return int(re.search(r'Ran (\d+) tests',output).group(1))
if args.mode=='stage':
    if BASELINE.exists() or VALIDATION.exists():raise ValueError('new_validation_required')
    for root in ROOTS.values():
        for path in {root/'AGENTS.md',root/'config/AGENTS.md',*(p/'AGENTS.md' for p in root.parents)}:
            if path.exists():raise ValueError('read_applicable_instructions_first:'+str(path))
    branch=subprocess.check_output(['git','-C',str(REPO),'branch','--show-current'],text=True).strip()
    if branch!='feat/handyman-phase5-grasping':raise ValueError('unexpected_branch')
    if sha(PATCH)!=args.archive_sha:raise ValueError('archive_hash_mismatch')
    expected={
        CHANGED[0]:read(BASE/'canned-grasp-robust-v2-20261023-deployment.json')['hashes'][CHANGED[0]],
        CHANGED[1]:read(BASE/'canned-carry-v1-20261023-deployment.json')['hashes'][CHANGED[1]],
        CHANGED[2]:read(BASE/'canned-place-v1-20261023-entry-update.json')['source_sha256']}
    for root in ROOTS.values():
        for name,digest in expected.items():
            if sha(root/name)!=digest:raise ValueError('previous_adapter_changed:'+str(root/name))
        for name in ADDED:
            if (root/name).exists():raise ValueError('new_file_already_exists:'+str(root/name))
    names=[p.relative_to(CODE).as_posix() for p in CODE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not p.name.endswith('.pyc')]
    cfg=read(CODE/'config/apple_transfer_auto_v1.json');registry=read(Path(cfg['registry']))
    external=[Path(cfg['environment']),Path(cfg['semantic_map']),Path(cfg['registry']),Path(registry['weights'])]
    baseline=dict(branch=branch,archive_sha256=args.archive_sha,
        frozen={label:{name:sha(root/name) for name in names if name not in CHANGED and (root/name).is_file()} for label,root in ROOTS.items()},
        previous={label:{name:sha(root/name) for name in CHANGED} for label,root in ROOTS.items()},
        external={str(p):sha(p) for p in external},robot_commands_sent=False)
    new_json(BASELINE,baseline)
    with tarfile.open(BASE/(PREFIX+'-before-update.tar.gz'),'x:gz') as archive:
        archive.add(BASELINE,arcname=BASELINE.name)
        for label,root in ROOTS.items():
            for name in CHANGED:archive.add(root/name,arcname=label+'/'+name)
    VALIDATION.mkdir();shutil.copytree(CODE,CANDIDATE,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    with tarfile.open(PATCH) as archive:
        if set(archive.getnames())!=set(FILES):raise ValueError('unexpected_archive_files')
        for member in archive.getmembers():
            if not member.isfile() or Path(member.name).name!=member.name:raise ValueError('unsafe_archive_member')
            destination=CANDIDATE/member.name
            with destination.open('wb' if member.name in CHANGED else 'xb') as f:f.write(archive.extractfile(member).read())
    for name in FILES:
        if name.endswith('.py'):ast.parse((CANDIDATE/name).read_text())
    subprocess.run(['bash','-n',str(CANDIDATE/'run_object_transfer.sh')],check=True)
    count=tests(CANDIDATE,VALIDATION/'candidate-tests.log')
    with (VALIDATION/'actual-recorded-replay.log').open('x') as f:
        proc=subprocess.run([sys.executable,str(BASE/'validate_can_continuous_integration_20261023.py'),
            '--candidate',str(CANDIDATE),'--output',str(VALIDATION/'actual-recorded-replay')],stdout=f,stderr=subprocess.STDOUT,timeout=180)
    if proc.returncode:
        print((VALIDATION/'actual-recorded-replay.log').read_text()[-6000:],flush=True);raise ValueError('actual_handoff_replay_failed')
    replay=read(VALIDATION/'actual-recorded-replay/result.json')
    with (VALIDATION/'offline-flow.log').open('x') as f:
        subprocess.run([sys.executable,str(CANDIDATE/'run_object_transfer.py'),'--object','canned_juice','--offline-smoke-test',
            '--output',str(VALIDATION/'offline-can')],stdout=f,stderr=subprocess.STDOUT,check=True,timeout=30)
    flow=read(VALIDATION/'offline-can/result.json')
    if flow['physical_transfer_verified'] or len(flow['confirmed_stages'])!=6:raise ValueError('offline_flow_claimed_physical_success')
    frozen(baseline);previous(baseline)
    result=dict(status='candidate_full_regression_actual_replay_and_flow_passed',candidate_tests=count,actual_replay=replay,
        hashes={name:sha(CANDIDATE/name) for name in FILES},archive_sha256=args.archive_sha,robot_commands_sent=False)
    new_json(VALIDATION/'result.json',result);print(json.dumps(result),flush=True)
else:
    if REPORT.exists():raise ValueError('new_deployment_required')
    baseline=read(BASELINE);tested=read(VALIDATION/'result.json')
    if tested['status']!='candidate_full_regression_actual_replay_and_flow_passed':raise ValueError('validated_candidate_required')
    sys.path.insert(0,str(CODE));from apple_transfer_automation import RobotLock
    with RobotLock():
        frozen(baseline);previous(baseline)
        for name,digest in tested['hashes'].items():
            if sha(CANDIDATE/name)!=digest:raise ValueError('candidate_changed')
        for root in ROOTS.values():
            for name in FILES:
                pending=root/(name+'.can-integration-pending')
                with pending.open('xb') as f:f.write((CANDIDATE/name).read_bytes())
                pending.replace(root/name)
            (root/'run_object_transfer.sh').chmod(0o755)
        frozen(baseline)
        for root in ROOTS.values():
            if any(sha(root/name)!=digest for name,digest in tested['hashes'].items()):raise ValueError('deployed_copy_mismatch')
        with (VALIDATION/'deployed-file-preflight.log').open('x') as f:
            subprocess.run([sys.executable,str(CODE/'run_object_transfer.py'),'--object','canned_juice','--check-inputs'],
                stdout=f,stderr=subprocess.STDOUT,check=True,timeout=45)
        name='canned-continuous-entry-ready-progress-20261023.json'
        if any((root/name).exists() for root in [BASE,*ROOTS.values()]):raise ValueError('new_checkpoint_required')
        checkpoint=dict(status='continuous_can_entry_code_ready_fresh_session_test_pending',saved_ns=time.time_ns(),
            branch=baseline['branch'],code_copies=3,tests=tested['candidate_tests'],actual_recorded_replay=tested['actual_replay'],
            files=FILES,hashes=tested['hashes'],deployment='D:/handyman-training-runs/'+REPORT.name,
            entry='/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/run_object_transfer.sh',
            task=dict(object='canned_juice',source='LayoutA bedroom white_side_table',destination='living_room square_low_table'),
            new_entry_uses_current_run_context_not_historical_checkpoints=True,
            parent_control_ownership_spans_all_stages=True,original_apple_sources_model_map_route_tolerances_unchanged=True,
            last_physical_result='D:/handyman-training-runs/canned-transfer-verified-progress-20261023.json',
            this_step_robot_commands_sent=False,new_continuous_live_trial_started=False,
            uninterrupted_full_can_transfer_verified=False,task_finished_sent=False,
            next=['Request Unity session1 fresh initial robot pose and upright canned_juice on bedroom white_side_table.',
                'After the user reports ready, execute ONE new run of run_object_transfer.sh --object canned_juice --run --confirm-layout LayoutA --confirm-localization --simulation-ignore-collision --output NEW_D_DIRECTORY.',
                'Monitor the unified journal and child logs. No navigation time limit. Never replay failed grasp/retraction/navigation/release; audit the physical state.',
                'Only a successful new continuous run may set uninterrupted_can_transfer_verified. Historical recorded replay is explicitly not live authorization.'])
        data=json.dumps(checkpoint,indent=2,allow_nan=False).encode()
        for root in [BASE,*ROOTS.values()]:
            with (root/name).open('xb') as f:f.write(data)
        result=dict(status='continuous_can_entry_deployed_ready_for_fresh_session_trial',candidate_tests=tested['candidate_tests'],
            files=FILES,hashes=tested['hashes'],checkpoint='D:/handyman-training-runs/'+name,checkpoint_sha256=sha(BASE/name),
            code_copies=3,checkpoint_copies=4,original_apple_sources_model_map_route_tolerances_unchanged=True,
            robot_commands_sent=False,continuous_physical_trial_verified=False)
        new_json(REPORT,result);print(json.dumps(result),flush=True)
