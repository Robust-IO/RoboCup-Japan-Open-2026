"""Verify the installed efficiency revision. No ROS init or robot commands."""
import hashlib,json,sys,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
sys.path[:0]=[str(b),str(f)]
receipt=json.loads((b/'navigation-efficiency-deployment-latest.json').read_text())
if receipt.get('rollback_completed'):raise ValueError('navigation_efficiency_has_been_rolled_back')
for row in receipt['files']:
    if row.get('metadata'):continue
    if hashlib.sha256(Path(row['target']).read_bytes()).hexdigest()!=row['sha256']:raise ValueError('installed_navigation_file_changed:'+row['target'])
progress=json.loads((b/'phase5-current-work-progress-20261026.json').read_text())
for name,digest in progress['latest_expected_hashes'].items():
    if hashlib.sha256((f/name).read_bytes()).hexdigest()!=digest:raise ValueError('source_entry_whitelist_mismatch:'+name)
from apple_transfer_automation import check_inputs
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text())
if cfg['navigation_motion_profile']!='efficiency_v1':raise ValueError('efficiency_navigation_not_selected')
audit=check_inputs(cfg)
from navigation_efficiency_policy import load_policy
from motion_speed import load_profile
import efficient_navigation
policy=load_policy();profile=load_profile('efficiency_v1')
out=b/'navigation-efficiency-checkpoint-20261106.json'
old=json.loads(out.read_text()) if out.exists() else {}
report=dict(status='installed_offline_verified',created_ns=time.time_ns(),backup=receipt['backup'],candidate=receipt['candidate'],
    tests=receipt['validation']['tests'],source_entry_hash_checks_passed=True,installed_files_verified=len(receipt['files']),
    navigation_motion_profile='efficiency_v1',policy=policy,profile=profile,robot_commands_sent=False,
    live_navigation_validated=False,full_live_transfer_validated=False,
    rollback_command='python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/rollback_navigation_efficiency_20261106.py --run',
    previous_successful_transfer='/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/cigarette-full-auto-20261106-200422',
    next='Read live state first. Robot is not assumed to be at initial spawn. Do not replay grasp/release.')
if old.get('candidate')==report['candidate']:
    for key in ('live_navigation_validated','full_live_transfer_validated','latest_live_navigation','latest_robot_state'):
        if key in old:report[key]=old[key]
out.write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('policy','profile')},ensure_ascii=False))
