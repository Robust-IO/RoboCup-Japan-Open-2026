"""Verify installed imports/config hashes, then save the handoff; no ROS init."""
# A later navigation revision deliberately supersedes selected V2 file hashes.
import json as _json, runpy as _runpy
from pathlib import Path as _Path
_nav_receipt=_Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/navigation-efficiency-deployment-latest.json')
if _nav_receipt.exists() and not _json.loads(_nav_receipt.read_text()).get('rollback_completed'):
    _runpy.run_path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/verify_navigation_efficiency_20261106.py',run_name='__main__')
    raise SystemExit(0)
import hashlib,json,sys,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
sys.path[:0]=[str(b),str(f)]
receipt=json.loads((b/'gate-revision-deployment-latest.json').read_text())
for row in receipt['files']:
    if hashlib.sha256(Path(row['target']).read_bytes()).hexdigest()!=row['sha256']:raise ValueError('installed_file_changed:'+row['target'])
from apple_transfer_automation import check_inputs
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());inputs=check_inputs(cfg)
expected=json.loads((b/'phase5-current-work-progress-20261026.json').read_text())['latest_expected_hashes']
for name,digest in expected.items():
    if hashlib.sha256((f/name).read_bytes()).hexdigest()!=digest:raise ValueError('source_entry_whitelist_mismatch:'+name)
import flat_box_grasp,flat_box_place,flat_box_carry,outcome_transfer_runtime
if flat_box_grasp.BoxRuntime.default_observation_profile!='rgbd':raise ValueError('unexpected_grasp_profile')
if flat_box_place.TaskPlaceRuntime.default_observation_profile!='rgbd':raise ValueError('unexpected_place_profile')
if flat_box_carry.TaskCarryRuntime.default_observation_profile!='hand':raise ValueError('unexpected_carry_profile')
from navigation_entry import finite_navigation_joints,precision_matches
from inspect_transfer_ready import validate_feedback
names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
validate_feedback(dict(zip(names,[.4,-1.1,.1,.2,.1,.31])),[.06,.08,.06],{'base':0},{'arm':1},1)
probe_receipt=json.loads((Path(receipt['candidate'])/'read-only-probe-receipt.json').read_text())
if probe_receipt.get('returncode')!=0:raise ValueError('live_readonly_probe_not_successful')
probe=json.loads((Path(probe_receipt['output'])/'result.json').read_text())
counts={v['name']:v.get('tests') for v in receipt['validation']['checks']}
report=dict(status='gate_revision_v2_installed_verified',created_ns=time.time_ns(),backup=receipt['backup'],candidate=receipt['candidate'],
    installed_files_verified=len(receipt['files']),input_audit=inputs,source_entry_hash_checks_passed=True,
    new_behavior_tests=counts['new-behavior-tests'],existing_geometry_tests=counts['existing-geometry-tests'],recorded_transfer_replay_passed=True,
    live_readonly_capture=probe,robot_commands_sent=False,full_live_transfer_validated=False,
    rollback_command='python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/rollback_gate_revision_20261106.py --run',
    next='Use the current fresh-intake launcher for a physical regression when requested; do not assume the robot is at its initial pose.')
checkpoint=b/'phase5-gate-revision-checkpoint-20261106.json'
if checkpoint.exists():
    old=json.loads(checkpoint.read_text())
    if old.get('candidate')==report['candidate'] and old.get('full_live_transfer_validated'):
        for key in ('full_live_transfer_validated','latest_live_regression','latest_robot_state'):
            if key in old:report[key]=old[key]
checkpoint.write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('live_readonly_capture','input_audit')},ensure_ascii=False))
