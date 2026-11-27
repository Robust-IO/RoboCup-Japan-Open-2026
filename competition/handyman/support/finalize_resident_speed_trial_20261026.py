"""Build a clean result/checkpoint only after the new trial has actually ended."""
import hashlib,json,sys,time,shutil
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'penguin-resident-fast-transfer-20261026-01'
repo=Path('/__HANDYMAN_PACKAGE__/project');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
result=json.loads((root/'result.json').read_text())
if 'completed_ns' not in result:raise ValueError('live_trial_still_running')
ready=json.loads((base/'penguin-resident-fast-ready-progress-20261026.json').read_text())
for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
 for name,expected in ready['latest_expected_hashes'].items():
  if hashlib.sha256((tree/name).read_bytes()).hexdigest()!=expected:raise ValueError('code_changed:'+str(tree/name))
benchmark=json.loads((base/'resident-model-benchmark-20261026-01.json').read_text())
rows=[json.loads(line) for line in (root/'journal.jsonl').read_text().splitlines()]
spans=[]
stages=[r for r in rows if r.get('event')=='stage']
for i,r in enumerate(stages):
 end=stages[i+1]['time_ns'] if i+1<len(stages) else result['completed_ns']
 spans.append(dict(stage=r['stage'],seconds=round((end-r['time_ns'])/1e9,3)))
checkpoint=dict(version=1,status='resident_model_fast_navigation_trial_recorded',saved_ns=time.time_ns(),
 class_name='toy_penguin',trial=str(root),tests_passed=ready['tests_passed'],latest_expected_hashes=ready['latest_expected_hashes'],
 three_deployed_copies_verified=True,model_weights_changed=False,model_benchmark=str(base/'resident-model-benchmark-20261026-01.json'),
 resident_reuse_verified=benchmark['same_model_pid'],warm_attach_wall_s=benchmark['rows'][1]['client_attach_wall_s'],
 navigation_profile='carry_fast_v1',long_segment_holding_linear_mps=.28,empty_angular_rps=.8,holding_angular_rps=.8,
 short_segment_linear_mps=.2,short_segment_angular_rps=.8,old_profile='balanced_v1',
 routes_goal_tolerances_arm_and_finite_approach_unchanged=True,
 result_status=result['status'],elapsed_s=result['elapsed_s'],stage_seconds=spans,
 initial_pose=result.get('initial_base_map_pose'),close_commands_sent=result['close_commands_sent'],release_commands_sent=result['release_commands_sent'],
 grasp_success_verified=result['grasp_success_verified'],placement_success_verified=result['placement_success_verified'],
 uninterrupted_transfer_verified=result.get('uninterrupted_transfer_verified',False),
 physical_transfer_verified=result['physical_transfer_verified'],robot_ready_for_next_task=result.get('robot_ready_for_next_task',False),
 last_fresh_robot_state=result.get('last_fresh_robot_state'),task_finished_sent=result['task_finished_sent'],
 current_object_location='square_low_table#living_room' if result['placement_success_verified'] else 'requires_actual_state_review',
 source_geometry_fix='depth_connected_foreground_excludes_background_pot_without_extent_clipping',
 mount_keeper_record=str(base/'drvfs-persistent-mount-20261026.json'),
 next='Review measured timing; use reverse task if another transport test is requested. Do not replay this completed task.',
 comparison_limit='Different objects/dynamic scenes; no controlled A/B speedup is claimed.')
if result['physical_transfer_verified']:
 sys.path.insert(0,str(base));from transfer_stage_timing import summarize
 checkpoint['timing']=summarize(root)
 checkpoint['status']='resident_model_and_fast_navigation_one_full_penguin_transfer_verified'
for name in ('source-navigation.json','destination-navigation.json'):
 p=root/name
 if p.exists():
  nav=json.loads(p.read_text());checkpoint[name]=dict(status=nav.get('status'),speed_contexts=nav.get('speed_contexts'),speed_parameters_restored=nav.get('speed_parameters_restored'),goal_precision_changed=nav.get('goal_precision_changed'),feedback_recoveries=nav.get('feedback_recoveries'))
notes=base/'resident-model-speed-notes-20261026.md'
if notes.exists():
 for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
  target=tree/'模型常驻与速度试验.md';shutil.copyfile(notes,target)
 checkpoint['latest_expected_hashes']['模型常驻与速度试验.md']=hashlib.sha256(notes.read_bytes()).hexdigest()
name='resident-model-fast-navigation-trial-progress-20261026.json'
for output in (base/name,repo/'training'/name):
 with output.open('x') as stream:json.dump(checkpoint,stream,indent=2)
print(json.dumps({k:checkpoint[k] for k in ('status','tests_passed','result_status','elapsed_s','uninterrupted_transfer_verified','physical_transfer_verified','robot_ready_for_next_task','current_object_location','navigation_profile','warm_attach_wall_s')}))
