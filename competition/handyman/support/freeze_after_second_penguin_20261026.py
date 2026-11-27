"""Record the tested configuration; do not modify runtime files or robot state."""
import hashlib,json,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
old=json.loads((base/'resident-model-fast-navigation-trial-progress-20261026.json').read_text())
comparison=json.loads((base/'resident-model-second-timing-comparison-20261026.json').read_text())
if not comparison['physical_transfer_verified'] or not comparison['uninterrupted_transfer_verified'] or not comparison['robot_ready_for_next_task']:
 raise ValueError('verified_complete_second_trial_required')
for root in (Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1'):
 for name,expected in old['latest_expected_hashes'].items():
  if hashlib.sha256((root/name).read_bytes()).hexdigest()!=expected:raise ValueError('configuration_changed:'+str(root/name))
record=dict(version=1,status='two_penguin_forward_runs_verified_current_configuration_frozen',saved_ns=time.time_ns(),
 latest_expected_hashes=old['latest_expected_hashes'],three_code_copies_verified=True,last_runtime_tests_passed=old['tests_passed'],
 navigation_profile='carry_fast_v1',holding_long_linear_mps=.28,empty_angular_rps=.8,holding_angular_rps=.8,short_linear_mps=.2,
 model_sha256=comparison['repeat_model']['model_sha256'],resident_model_pid=comparison['repeat_model']['model_pid'],
 first_trial=comparison['first']['run'],first_seconds=comparison['first']['elapsed_s'],
 second_trial=comparison['repeat']['run'],second_seconds=comparison['repeat']['elapsed_s'],
 second_timing=comparison['repeat']['seconds_by_group'],both_runs_were_warm=True,
 user_decision='No further performance tuning after repeat passes. Keep fixed route and proceed with another object.',
 known_limitation='Penguin reverse-restoration stopped after one close because a yellow background produced a second hand-reference component; excluded from successful forward count. Not fixed during timing comparison.',
 last_confirmed_robot_state=comparison['final_robot_state'],last_confirmed_object_location='square_low_table#living_room',
 next_trial='pepper-frozen-cross-object-transfer-20261026-01',
 next_scope='Body-grasp ground_pepper on the same furniture route under the frozen current model and speed profile; actual position comes only from new RGBD.')
for path in (base/'phase5-frozen-configuration-progress-20261026.json',repo/'training/phase5-frozen-configuration-progress-20261026.json'):
 with path.open('x') as stream:json.dump(record,stream,indent=2)
print(json.dumps({k:record[k] for k in ('status','first_seconds','second_seconds','resident_model_pid','next_trial')}))
