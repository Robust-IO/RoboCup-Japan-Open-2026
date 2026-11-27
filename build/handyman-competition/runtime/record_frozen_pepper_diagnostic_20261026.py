"""Preserve passed timing and the subsequent unfinished object-validation case."""
import hashlib,json,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
frozen=json.loads((base/'phase5-frozen-configuration-progress-20261026.json').read_text())
root=base/'pepper-frozen-cross-object-transfer-20261026-01';r=json.loads((root/'result.json').read_text())
if 'completed_ns' not in r or r['close_commands_sent']!=0 or r['release_commands_sent']!=0:
 raise ValueError('review_current_state_before_recording_preclose_stop')
for tree in (Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1'):
 for name,expected in frozen['latest_expected_hashes'].items():
  if hashlib.sha256((tree/name).read_bytes()).hexdigest()!=expected:raise ValueError('frozen_configuration_changed:'+str(tree/name))
model=json.loads((root/'grasp/model-worker.log').read_text())
record=dict(version=1,status='penguin_repeat_passed_configuration_frozen_next_object_alignment_under_investigation',saved_ns=time.time_ns(),
 latest_expected_hashes=frozen['latest_expected_hashes'],last_code_tests_passed=frozen['last_runtime_tests_passed'],runtime_code_and_configuration_unchanged=True,
 successful_penguin_repeat_seconds=frozen['second_seconds'],successful_penguin_first_seconds=frozen['first_seconds'],
 successful_penguin_forward_count=2,active_object='ground_pepper',active_trial=str(root),
 active_trial_status=r['status'],active_trial_stage=r['stage'],active_trial_reason=r.get('reason'),
 active_trial_elapsed_s=r['elapsed_s'],active_trial_completed_transfer=False,
 close_commands_sent=r['close_commands_sent'],release_commands_sent=r['release_commands_sent'],
 last_confirmed_robot_state=r.get('last_fresh_robot_state'),current_robot_description='empty open hand, arm extended, in bedroom at source table; no running task',
 penguin_last_placed_location='square_low_table#living_room',pepper_last_observed_location='bedroom source table; inspect new RGBD before movement',
 model_pid=model.get('model_pid'),resident_model_reused=model.get('resident_reused'),
 pepper_source_detection_confidence=[o['detection']['confidence'] for o in r.get('source_observations',[])],
 observation='Approach reaches the commanded odometry endpoints within about1.5mm. After forward moves, near RGBD lateral error changes to about-23mm or-17mm; first close view visibly shows tilted pepper. Controller retreats to150mm and consumes the six approach actions before closing.',
 root_cause_not_yet_proven='Need distinguish finger contact/object displacement from near RGBD/transform bias; do not just loosen closure checks or claim model confidence failure.',
 diagnostic=str(root/'alignment-audit.json'),
 next='Continue fixed-route object validation by auditing near-target alignment; keep resident model, speed profile, route and precision fixed. Reacquire actual state before any continuation. No grasp/release replay.',
 known_penguin_reverse_limitation='Yellow background caused nonunique hand-reference component in an earlier restoration, excluded from successful forward count.',
 phase5_statistical_acceptance_complete=False)
name='phase5-current-work-progress-20261026.json'
for p in (base/name,repo/'training'/name):
 with p.open('x') as f:json.dump(record,f,indent=2)
print(json.dumps({k:record[k] for k in ('status','runtime_code_and_configuration_unchanged','successful_penguin_repeat_seconds','active_trial_status','active_trial_reason','close_commands_sent','current_robot_description')}))
