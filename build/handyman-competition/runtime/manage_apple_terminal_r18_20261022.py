"""Audited code deployment/replay ONLY. Never invokes a robot action entrypoint."""
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

P=argparse.ArgumentParser();P.add_argument('mode',choices=['audit','stage','deploy']);A=P.parse_args()
RUNS=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');REPO=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
ROOTS=dict(runtime=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),training=REPO/'training',
           module=REPO/'training/automation/apple_transfer_v1')
CODE=ROOTS['runtime'];PREFIX='apple-terminal-r18-20261022'
PATCH=RUNS/(PREFIX+'-patch.tar.gz')
EXPECTED='86de6f6f4d95d2b70c9359e4a198e814938ed18c1305dd9dbbda6320118c8f5a'
FILES=['run_bedroom_apple_regrasp.py','apple_approach_lease.py','reviewed_carry_adjustment.py',
       'grasp_terminal_correction.py','test_grasp_terminal_correction.py','APPLE_TRANSFER_AUTOMATION.txt']
FROZEN=['navigate_carry_trial.py','motion_speed.py','config/motion_speed_profiles.json','config/apple_transfer_auto_v1.json',
        'apple_transfer_auto_backend.py','apple_transfer_automation.py','pregrasp_raised_trial.py',
        'run_apple_close_lift_trial.py','task_outcome_policy.py','table_release_gate.py','verify_table_placement.py',
        'capture_grasp_scene.py','apple_shared_inference.py','apple_inference_worker.py']
BASELINE=RUNS/(PREFIX+'-baseline.json');VALIDATION=RUNS/(PREFIX+'-validation-01')
REPORT=RUNS/(PREFIX+'-deployment.json');CHECKPOINT='apple-terminal-r18-progress-20261022.json'
POINTERS=['phase5-visual-test-preparation-20261022.json','phase5-restart-checkpoint-20261017.json',
          'apple-transfer-automation-progress-20261017.json','apple-transfer-live-progress-20261017.json',
          'apple-outcome-policy-progress-20261018.json','apple-efficiency-r17-progress-20261022.json']
def read(path):return json.loads(path.read_text())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
def write_new(path,value):
    with path.open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False)
def ensure_baseline(base):
    for label,root in ROOTS.items():
        for name,expected in {**base['hashes'][label],**base['frozen'][label]}.items():
            if sha(root/name)!=expected:raise RuntimeError('concurrent_source_change:'+label+'/'+name)
def test_tree(root,log):
    env=os.environ.copy();env['PYTHONPATH']=str(root)+':'+str(root/'scripts')
    with log.open('x') as stream:
        code=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(root),'-p','test_*.py'],
            cwd=root,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=180).returncode
    text=log.read_text();print(text[-3000:],flush=True)
    if code:raise RuntimeError('tests_failed:'+str(log))
    return int(re.search(r'Ran (\d+) tests',text).group(1))

if A.mode=='audit':
    if BASELINE.exists():raise RuntimeError('baseline_exists')
    for root in ROOTS.values():
        for path in [root/'AGENTS.md',root/'config/AGENTS.md',*(p/'AGENTS.md' for p in root.parents)]:
            if path.exists():
                print(str(path)+'\n'+path.read_text());raise RuntimeError('read_instructions_first')
    deployed=read(RUNS/'apple-efficiency-r17-20261022-deployment.json')['hashes']
    for root in ROOTS.values():
        for name in FILES:
            expected=deployed.get(name)
            if sha(root/name)!=expected:raise RuntimeError('unexpected_prepatch_file:'+str(root/name))
    base=dict(files=FILES,hashes={k:{n:sha(root/n) for n in FILES} for k,root in ROOTS.items()},
        frozen={k:{n:sha(root/n) for n in FROZEN} for k,root in ROOTS.items()},
        branch=subprocess.check_output(['git','-C',str(REPO),'branch','--show-current'],text=True).strip(),
        robot_commands_sent=False)
    if base['branch']!='feat/handyman-phase5-grasping':raise RuntimeError('unexpected_branch')
    with tarfile.open(RUNS/'apple-before-terminal-r18-20261022.tar.gz','x:gz') as archive:
        for label,root in ROOTS.items():
            for name in FILES:
                if (root/name).exists():archive.add(root/name,arcname=label+'/'+name)
    write_new(BASELINE,base)
    print(json.dumps(dict(status='audited_backed_up',files=FILES,branch=base['branch'],robot_commands_sent=False)))

elif A.mode=='stage':
    base=read(BASELINE);ensure_baseline(base)
    if sha(PATCH)!=EXPECTED:raise RuntimeError('patch_hash_mismatch')
    VALIDATION.mkdir(exist_ok=False);candidate=VALIDATION/'candidate'
    shutil.copytree(CODE,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc','.git'))
    with tarfile.open(PATCH) as archive:
        if set(archive.getnames())!=set(FILES):raise RuntimeError('unexpected_patch_file_set')
        for member in archive.getmembers():
            if not member.isfile() or Path(member.name).is_absolute() or '..' in Path(member.name).parts:
                raise RuntimeError('unsafe_patch_member')
            with (candidate/member.name).open('wb') as stream:stream.write(archive.extractfile(member).read())
    for name in FILES:
        if name.endswith('.py'):ast.parse((candidate/name).read_text(),filename=name)
    for name,expected in base['frozen']['runtime'].items():
        if sha(candidate/name)!=expected:raise RuntimeError('frozen_file_modified:'+name)
    tests=test_tree(candidate,VALIDATION/'all-tests.log')
    sys.path.insert(0,str(candidate))
    from near_apple_geometry import recorded_candidate
    from apple_approach_lease import closure_position_error
    from grasp_terminal_correction import terminal_delta
    from hand_camera_ray import rotation
    from PIL import Image
    import numpy as np
    failed=RUNS/'automatic-apple-r17-20261022-01'
    diagnosis=read(failed/'stopped-readonly-diagnosis.json')
    frame=failed/'source-observe'/diagnosis['rows'][-1]['capture']
    observation=read(frame/'observation.json');snapshot=read(frame/'snapshot.json')
    candidate_geometry=recorded_candidate(observation,snapshot,np.asarray(Image.open(frame/'rgb.png').convert('RGB')),
        np.load(frame/'depth_m.npy',allow_pickle=False),directory=frame.parent)
    target=np.array(candidate_geometry['center_m']);height=diagnosis['grasp_plan']['palm_height_m']
    try:closure_position_error(target,height)
    except ValueError:old_gate_rejects=True
    else:old_gate_rejects=False
    if not old_gate_rejects:raise RuntimeError('failure_case_not_reproduced')
    delta=terminal_delta(target,height)
    predicted=target-np.array(delta);closure_position_error(predicted,height)
    replay=dict(status='recorded_failure_generates_combined_correction',capture=str(frame),
        original_pixels_refitted=True,target_base_m=target.tolist(),correction_delta_m=delta,
        predicted_after_correction=predicted.tolist(),predicted_only_not_actual_motion=True,
        actual_correction_verified=False,old_closure_gate_reproduced=True,
        robot_commands_sent=False,source_recordings_unchanged=True)
    write_new(VALIDATION/'recorded-replay.json',replay)
    result=dict(status='candidate_tests_and_recorded_regression_passed',revision='terminal-correction-r18',
        candidate=str(candidate),tests=tests,archive_sha256=EXPECTED,
        candidate_hashes={name:sha(candidate/name) for name in FILES},
        replay=replay,navigation_and_other_frozen_files_unchanged=True,robot_commands_sent=False,new_live_trial=False)
    write_new(VALIDATION/'result.json',result);print(json.dumps(result,indent=2),flush=True)

elif A.mode=='deploy':
    base=read(BASELINE);tested=read(VALIDATION/'result.json');candidate=Path(tested['candidate'])
    if REPORT.exists() or any((root/CHECKPOINT).exists() for root in [RUNS,*ROOTS.values()]):raise RuntimeError('deployment_exists')
    if tested['status']!='candidate_tests_and_recorded_regression_passed':raise RuntimeError('tested_candidate_required')
    actions={'apple_transfer_automation.py','pregrasp_raised_trial.py','reviewed_carry_adjustment.py',
             'run_bedroom_apple_regrasp.py','run_apple_close_lift_trial.py','continue_apple_placement.py'}
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:argv=[v.decode(errors='replace') for v in path.read_bytes().split(b'\0') if v]
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
        if '--run' in argv and any(Path(value).name in actions for value in argv):raise RuntimeError('active_action_prevents_deployment')
    sys.path.insert(0,str(CODE))
    from apple_transfer_automation import RobotLock,write_json
    with RobotLock():
        ensure_baseline(base)
        for name,expected in tested['candidate_hashes'].items():
            if sha(candidate/name)!=expected:raise RuntimeError('candidate_changed')
        roots=dict(runs=RUNS,**ROOTS)
        for name in POINTERS:
            if len({sha(root/name) for root in roots.values()})!=1:raise RuntimeError('metadata_disagrees')
        with tarfile.open(RUNS/'apple-before-terminal-r18-20261022-checkpoints.tar.gz','x:gz') as archive:
            for label,root in roots.items():
                for name in POINTERS:archive.add(root/name,arcname=label+'/'+name)
        for root in ROOTS.values():
            for name in FILES:
                path=root/name;temp=path.with_name(path.name+'.r18-deploy-pending')
                shutil.copyfile(candidate/name,temp);temp.replace(path)
        for label,root in ROOTS.items():
            for name,expected in base['frozen'][label].items():
                if sha(root/name)!=expected:raise RuntimeError('frozen_code_changed')
            for name,expected in tested['candidate_hashes'].items():
                if sha(root/name)!=expected:raise RuntimeError('source_copy_mismatch')
        tests=test_tree(CODE,RUNS/(PREFIX+'-runtime-tests.log'))
        old=read(RUNS/'apple-r17-full-live-progress-20261022.json')
        instruction=('Read this checkpoint FIRST for CURRENT CODE: terminal-correction-r18 is deployed. '
            'Efficient source grasp now does one full approach, then up to2 NEW measured XY corrections '
            'using strictly post-completion RGBD and successful same-task predecessors; it does not replay failed actions. '
            'Original close/contact/lift/place gates, navigation, speeds and maps unchanged. '
            'No NEW robot action or live correction has been executed by this patch. '
            'Last physical state remains r17 STOP: EMPTY/OPEN/EXTENDED in bedroom at source, not full-entry ready. '
            'Do not resume/replay r17 logs or treat its old timestamp-less motion receipt as a new correction authorization. '
            'Next motion requires a fresh explicitly authorized trial and current sensor/arm/localization audit. '
            'The recorded failure now yields a combined26.4mm forward/19.6mm lateral correction in replay; '
            'that predicted endpoint is NOT evidence of actual execution or completed grasp/placement.')
        progress=dict(status='terminal_correction_deployed_pending_live_trial',revision='terminal-correction-r18',
            saved_ns=time.time_ns(),branch=base['branch'],deployment=str(REPORT),tests=tests,
            validation=str(VALIDATION/'result.json'),backup=str(RUNS/'apple-before-terminal-r18-20261022.tar.gz'),
            previous_live_checkpoint='D:/handyman-training-runs/apple-r17-full-live-progress-20261022.json',
            current_robot=old['current_robot'],replay=tested['replay'],maximum_terminal_corrections=2,
            navigation_speed_close_and_place_gates_unchanged=True,robot_commands_sent=False,
            actual_terminal_correction_verified=False,whole_transfer_verified=False,
            task_finished_sent=False,git_commit_or_push_performed=False,next=instruction)
        pointer='D:/handyman-training-runs/'+CHECKPOINT;updates={CHECKPOINT:progress}
        for name in POINTERS:
            value=read(RUNS/name);value.update(current_state_authoritative_checkpoint=pointer,
                latest_validation_checkpoint=pointer,latest_terminal_correction_checkpoint=pointer,
                current_code_revision='terminal-correction-r18',older_current_fields_are_historical=True,next=instruction)
            if name=='apple-transfer-automation-progress-20261017.json':
                value.update(implementation_revision='20261022-terminal-correction-r18-on-efficiency-r17',
                    latest_patch='D:/handyman-training-runs/'+PREFIX+'-patch.tar.gz',latest_patch_sha256=EXPECTED,
                    development_status=progress['status'],latest_code_policy_revision=progress['status'])
            updates[name]=value
        for root in roots.values():
            for name,value in updates.items():write_json(root/name,value)
        hashes={name:sha(RUNS/name) for name in updates}
        if any(sha(root/name)!=digest for root in roots.values() for name,digest in hashes.items()):raise RuntimeError('metadata_sync_mismatch')
        result=dict(status='deployed_tests_passed',revision='terminal-correction-r18',tests=tests,
            files=FILES,hashes=tested['candidate_hashes'],checkpoint_hashes=hashes,checkpoint=pointer,
            navigation_and_frozen_files_unchanged=True,robot_commands_sent=False,new_live_trial=False)
        write_new(REPORT,result);print(json.dumps(result,indent=2),flush=True)
