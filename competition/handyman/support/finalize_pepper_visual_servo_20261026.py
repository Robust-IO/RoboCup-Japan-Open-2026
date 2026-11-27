"""Record independently verified transfer and changed grasp logic accurately."""
import hashlib,json,sys,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
root=base/'pepper-motion-surface-full-transfer-20261026-01';r=json.loads((root/'result.json').read_text())
if 'completed_ns' not in r or not r.get('physical_transfer_verified') or not r.get('robot_ready_for_next_task'):
 raise ValueError('complete_actual_transfer_required')
ready=json.loads((base/'pepper-motion-surface-ready-progress-20261026.json').read_text())
frozen=json.loads((base/'phase5-frozen-configuration-progress-20261026.json').read_text())
for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
 for n,h in ready['latest_expected_hashes'].items():
  if hashlib.sha256((tree/n).read_bytes()).hexdigest()!=h:raise ValueError('changed_source:'+str(tree/n))
for n in ('config/motion_speed_profiles.json','config/apple_transfer_auto_v1.json','motion_speed.py','navigate_carry_trial.py'):
 if ready['latest_expected_hashes'][n]!=frozen['latest_expected_hashes'][n]:raise ValueError('frozen_performance_setting_changed:'+n)
model=json.loads((root/'grasp/model-worker.log').read_text())
if model['model_sha256']!=frozen['model_sha256']:raise ValueError('model_weights_changed')
sys.path.insert(0,str(base));from transfer_stage_timing import summarize
old_attempts=[]
for name in ('pepper-frozen-cross-object-transfer-20261026-01','pepper-compact-realign-open-continuation-20261026-01','pepper-compact-realign-full-transfer-20261026-01','pepper-visual-motion-full-transfer-20261026-01','pepper-motion-direction-full-transfer-20261026-01'):
 d=json.loads((base/name/'result.json').read_text());old_attempts.append(dict(run=name,status=d['status'],reason=d.get('reason'),close_commands=d['close_commands_sent'],release_commands=d['release_commands_sent'],physical_transfer_verified=d['physical_transfer_verified']))
rows=[json.loads(l) for l in (root/'journal.jsonl').read_text().splitlines()]
record=dict(version=1,status='pepper_visual_motion_alignment_and_complete_transfer_verified',saved_ns=time.time_ns(),
 latest_expected_hashes=ready['latest_expected_hashes'],tests_passed=ready['tests_passed'],three_deployed_copies_verified=True,
 model_resident_preserved=model.get('resident_model'),resident_model_pid=model.get('model_pid'),model_sha256=model['model_sha256'],
 navigation_speed_route_precision_and_arm_speed_unchanged=True,performance_profile='carry_fast_v1',
 change_scope='ground_pepper finite visual servo: compact realign, static RGBD ROI/motion prediction, direction compensation, real partial curved-surface reacquisition; shared can controller hook defaults to identity for other objects',
 successful_trial=str(root),elapsed_s=r['elapsed_s'],timing=summarize(root),
 initial_base_map_pose=r.get('initial_base_map_pose'),start_is_current_source_room_pose_not_unity_spawn=True,
 physical_transfer_verified=r['physical_transfer_verified'],uninterrupted_transfer_verified=r.get('uninterrupted_transfer_verified'),
 close_commands_sent=r['close_commands_sent'],release_commands_sent=r['release_commands_sent'],
 approaches=[{k:a[k] for k in ('index','phase','delta','desired_visual_delta_m','controller_frame_angle_compensation_rad') if k in a} for a in r.get('approaches',[])],
 visual_prediction_count=sum(e.get('event')=='pepper_static_background_search_prediction' for e in rows),
 current_visual_motion_is_only_roi_prediction=True,grasp_and_final_placement_require_actual_current_depth=True,
 current_object_locations={'ground_pepper':'square_low_table#living_room','toy_penguin':'square_low_table#living_room'},
 last_fresh_robot_state=r.get('last_fresh_robot_state'),robot_ready_for_next_task=r.get('robot_ready_for_next_task'),
 previous_failed_prefixes_not_counted_as_success=old_attempts,
 task_finished_sent=r.get('task_finished_sent'),statistical_success_rate_established=False,
 known_unresolved='Penguin reverse restoration yellow-feature ambiguity remains separately recorded; not changed in this pepper fix.',
 next='Revalidate additional starting poses/objects with the retained speed profile. Both penguin and pepper are now on living-room square_low_table. Inspect new state and choose an appropriate source task before another movement; never replay an old held context.')
name='phase5-pepper-visual-servo-success-progress-20261026.json'
for p in (base/name,repo/'training'/name):
 with p.open('x') as f:json.dump(record,f,indent=2)
# Preserve the prior failure record, then make the current-work file truly current.
for directory in (base,repo/'training'):
 p=directory/'phase5-current-work-progress-20261026.json'
 backup=directory/'phase5-before-pepper-visual-servo-progress-20261026.json'
 if p.exists() and not backup.exists():backup.write_bytes(p.read_bytes())
 pending=p.with_suffix('.pending');pending.write_text(json.dumps(record,indent=2));pending.replace(p)
print(json.dumps({k:record[k] for k in ('status','tests_passed','elapsed_s','physical_transfer_verified','uninterrupted_transfer_verified','close_commands_sent','release_commands_sent','robot_ready_for_next_task','current_object_locations')}))
print(json.dumps(record['timing']['seconds_by_group']))
