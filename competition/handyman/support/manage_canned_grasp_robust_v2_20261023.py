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
BASE=Path('/__HANDYMAN_PACKAGE__/runtime');REPO=Path('/__HANDYMAN_PACKAGE__/project')
ROOTS=dict(runtime=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),training=REPO/'training',module=REPO/'training/automation/apple_transfer_v1')
CODE=ROOTS['runtime'];PREFIX='canned-grasp-robust-v2-20261023'
FILES=['canned_grasp_geometry.py','canned_grasp_runtime.py','run_canned_grasp_trial.py','test_canned_grasp_geometry.py','CANNED_GRASP_TRIAL.txt']
PATCH=BASE/(PREFIX+'-patch.tar.gz');EXPECTED='2286ccf45150208bb8f754bcfcab03bfe39271832693b84b2f9f503e9ec9486b'
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
    prior=read(BASE/'canned-grasp-v1-20261023-deployment.json')
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
        checkpoint='canned-grasp-lift-live-progress-20261023.json'
        if any((root/checkpoint).exists() for root in [BASE,*ROOTS.values()]):raise ValueError('new_progress_required')
        held=read(BASE/'canned-current-held-verification-20261023-01/result.json')
        if held.get('grasp_lift_verified') is not True or held.get('robot_commands_sent') is not False:raise ValueError('current_readonly_held_verification_required')
        last_held=held['rows'][-1]
        held_snapshot=read(Path(last_held['capture'])/'snapshot.json')
        held_offset=[float(a-b) for a,b in zip(last_held['measurement']['center_m'],held_snapshot['palm_base_at_depth'])]
        progress=dict(status='one_can_grasp_and_lift_verified_not_yet_carried_or_placed',saved_ns=time.time_ns(),
            branch=baseline['branch'],scope='One simulated upright canned_juice grasp/lift, NOT a completed transfer or success-rate estimate.',
            can_grasp_and_lift_verified=True,can_carry_and_placement_verified=False,generic_full_transfer_enabled=False,
            source_approach='D:/handyman-training-runs/canned-grasp-v1-live-20261023-01',
            actual_close_and_lift='D:/handyman-training-runs/canned-close-lift-continuation-20261023-01',
            independent_all_recorded_frames_verification=str(VALIDATION/'recorded-checks/result.json'),
            current_held_verification='D:/handyman-training-runs/canned-current-held-verification-20261023-01/result.json',
            maximum_new_measured_corrections=5,total_coarse_approaches=1,total_close_commands=1,total_lifts=1,
            preparation_or_approach_replayed=False,hand_released=False,
            held_opening_rad=held['held_opening_rad'],last_measured_robot_state=held['last_fresh_robot_state'],
            held_center_minus_palm_base_m=held_offset,
            held_center_minus_palm_odom_m=last_held['center_minus_palm_odom_m'],
            estimated_upright_bottom_below_palm_m=.12506792828065508/2-held_offset[2],
            state_verified_at_ns=held['last_fresh_robot_state']['stamp_ns'],boot_id=held['boot_id'],
            current_pose_is_not_a_new_motion_lease=True,
            can_height_m=.12506792828065508,can_footprint_radius_m=.03767738938288809,
            cylinder_half_height_is_not_radius=True,held_upright_observation_strategy='actual_visible_upper_side_70_to_95_percent_and_cap',
            apple_legacy_entry_unchanged=True,model_or_navigation_changed=False,
            runtime_tests=count,candidate_tests=tested['candidate_tests'],recorded_replay=tested['replay'],
            deployment=str(REPORT),files=FILES,robot_commands_sent=True,task_finished_sent=False,
            next='The can remains held in the extended horizontal arm after one real10cm verification lift. Do NOT re-run the full grasp entry, reopen, re-close or repeat the approach. Reacquire fresh RGBD/joints before any next action. Next develop can-specific held tracking through folding and use the original navigation; placement must use cylinder half-height rather than horizontal radius for bottom clearance. No carry/placement verified yet. Single-frame near failures now reacquire NEW depth only (max5), never replay a command. Original failed verdicts remain preserved as diagnostic records; independent evidence and current held depth establish success.')
        for root in [BASE,*ROOTS.values()]:write_json(root/checkpoint,progress)
        result=dict(status='can_observation_addon_update_verified',files=FILES,hashes=tested['hashes'],
            runtime_tests=count,candidate_tests=tested['candidate_tests'],original_apple_runtime_preserved=True,
            source_copies=3,checkpoint_copies=4,checkpoint='D:/handyman-training-runs/'+checkpoint,
            checkpoint_sha256=sha(BASE/checkpoint),robot_commands_sent=False,can_live_mover_complete=False)
        new_json(REPORT,result);print(json.dumps(result),flush=True)
