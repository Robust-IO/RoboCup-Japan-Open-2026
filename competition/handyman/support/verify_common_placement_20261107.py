"""Check shared placement source consistency and imports; never starts ROS."""
import hashlib,json,os,subprocess,sys,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
receipt=json.loads((b/'common-placement-deployment-latest.json').read_text())
if receipt.get('rollback_completed'):raise ValueError('common_placement_already_rolled_back')
for row in receipt['files']:
    if row.get('metadata'):continue
    if hashlib.sha256(Path(row['target']).read_bytes()).hexdigest()!=row['sha256']:raise ValueError('installed_source_changed:'+row['target'])
progress=json.loads((b/'phase5-current-work-progress-20261026.json').read_text())
for name,digest in progress['latest_expected_hashes'].items():
    if hashlib.sha256((f/name).read_bytes()).hexdigest()!=digest:raise ValueError('source_entry_whitelist_mismatch:'+name)
installed={row['name']:row for row in json.loads((b/'outcome-policy-deployment.json').read_text())['installed']}
for row in receipt['files']:
    if row['relative'].startswith('runtime/') and installed[Path(row['target']).name]['sha256']!=row['sha256']:
        raise ValueError('runtime_deployment_metadata_mismatch')
code="""
from table_transfer_core import release_height,lift_targets
from stopped_capture_runtime import with_stopped_capture
from pepper_runtime import PepperRuntime
from penguin_runtime import PenguinRuntime
from outcome_transfer_runtime import OutcomeRuntime
from pathlib import Path
import outcome_transfer_runtime
for cls in (PepperRuntime,PenguinRuntime):
    wrapped=with_stopped_capture(cls)
    assert issubclass(wrapped,OutcomeRuntime)
    assert wrapped.move_joints is cls.move_joints
assert Path(outcome_transfer_runtime.__file__).with_name('outcome_finite_adjustment.py').exists()
assert lift_targets(release_height(.75,.03,.02))['release_lift_m']>.443
"""
for paths in ((f,b),(b,f)):
    env=dict(os.environ,PYTHONPATH=':'.join(map(str,paths)))
    subprocess.run([sys.executable,'-c',code],env=env,check=True,cwd='/tmp')
sys.path[:0]=[str(b),str(f)]
from apple_transfer_automation import check_inputs
cfg=json.loads((f/'config/apple_transfer_auto_v1.json').read_text());audit=check_inputs(cfg)
report=dict(status='common_table_transfer_v1_installed_verified',created_ns=time.time_ns(),candidate=receipt['candidate'],
    backup=receipt['backup'],files_verified=len(receipt['files']),offline_tests=receipt['validation']['tests'],
    recorded_replay=receipt['validation']['recorded_replay'],both_import_orders_passed=True,source_entry_hash_checks_passed=True,
    robot_commands_sent=False,full_live_transfer_validated=False,all_30_categories_physically_tested=False,
    previous_actual_dynamic_height_transfer=str(b/'pepper-kitchen-dynamic-continue-20261107-155837'),
    next='Uninterrupted physical regression from an explicitly observed current state, not an assumed spawn.')
(b/'common-placement-checkpoint-20261107.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report))
