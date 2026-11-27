"""Record one further object test under the exact frozen pipeline."""
import hashlib,json,sys,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
sys.path.insert(0,str(base));from transfer_stage_timing import summarize
frozen=json.loads((base/'phase5-frozen-configuration-progress-20261026.json').read_text())
root=base/'pepper-frozen-cross-object-transfer-20261026-01';r=json.loads((root/'result.json').read_text())
if 'completed_ns' not in r:raise ValueError('trial_not_finished')
if r.get('physical_transfer_verified') is not True or r.get('robot_ready_for_next_task') is not True:
 raise ValueError('actual_state_review_required_before_recording_success')
for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
 for name,expected in frozen['latest_expected_hashes'].items():
  if hashlib.sha256((tree/name).read_bytes()).hexdigest()!=expected:raise ValueError('frozen_pipeline_changed:'+str(tree/name))
model=json.loads((root/'grasp/model-worker.log').read_text())
second=json.loads((base/'resident-model-second-timing-comparison-20261026.json').read_text())
report=dict(version=1,status='frozen_profile_cross_object_full_transfer_verified',saved_ns=time.time_ns(),
 frozen_configuration=str(base/'phase5-frozen-configuration-progress-20261026.json'),
 latest_expected_hashes=frozen['latest_expected_hashes'],runtime_code_and_parameters_unchanged=True,
 last_full_code_tests_passed=frozen['last_runtime_tests_passed'],three_code_copies_verified=True,
 model_sha256=model['model_sha256'],resident_model_pid=model['model_pid'],resident_reused=model['resident_reused'],
 same_resident_pid_as_penguin=model['model_pid']==frozen['resident_model_pid'],navigation_profile='carry_fast_v1',
 fixed_route='LayoutA bedroom white_side_table#bedroom_1 to living_room square_low_table#living_room',
 current_frozen_profile_evidence={
  'toy_penguin':dict(complete_forward_runs=2,seconds=[second['first']['elapsed_s'],second['repeat']['elapsed_s']],grasp_part='head',reverse_run_verified=False),
  'ground_pepper':dict(complete_forward_runs=1,seconds=[r['elapsed_s']],grasp_part='body',prior_three_other_profile_runs_not_counted_here=True),
  'apple':dict(current_profile_verified=False,previous_profile_full_runs_exist=True),
  'canned_juice':dict(current_profile_verified=False,previous_profile_full_runs_exist=True),
  'rubiks_cube':dict(current_profile_verified=False,layout_a_native_available=False)},
 pepper_result=str(root/'result.json'),pepper_timing=summarize(root),
 pepper_close_commands=r['close_commands_sent'],pepper_release_commands=r['release_commands_sent'],
 pepper_uninterrupted_transfer_verified=r.get('uninterrupted_transfer_verified',False),
 physical_transfer_verified=r['physical_transfer_verified'],robot_ready_for_next_task=r['robot_ready_for_next_task'],
 last_confirmed_robot_state=r.get('last_fresh_robot_state'),
 current_object_locations={'toy_penguin':'square_low_table#living_room','ground_pepper':'square_low_table#living_room'},
 known_unresolved='Penguin reverse restoration failed on nonunique yellow hand reference; excluded from successful tests. No performance/verification changes made during this user-requested freeze.',
 competition_protocol_success_sent=r.get('task_finished_sent'),
 statistical_success_rate_established=False,
 next_scope='Continue fixed-route object/pose coverage before randomizing routes; current evidence covers head-grasp and body-grasp objects under this frozen profile, not all30classes or phase5 statistical acceptance.')
name='phase5-frozen-multiobject-validation-progress-20261026.json'
for p in (base/name,repo/'training'/name):
 with p.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:report[k] for k in ('status','runtime_code_and_parameters_unchanged','same_resident_pid_as_penguin','pepper_close_commands','pepper_release_commands','physical_transfer_verified','robot_ready_for_next_task','current_object_locations')}))
print(json.dumps(report['pepper_timing']))
