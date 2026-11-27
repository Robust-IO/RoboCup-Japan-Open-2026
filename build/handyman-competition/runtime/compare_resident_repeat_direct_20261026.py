import json,sys,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');sys.path.insert(0,str(base))
from transfer_stage_timing import summarize
first=base/'penguin-resident-fast-transfer-20261026-01';repeat=base/'penguin-resident-fast-transfer-20261026-02'
old=summarize(first);new=summarize(repeat)
prior_model=json.loads((first/'grasp/model-worker.log').read_text());current_model=json.loads((repeat/'grasp/model-worker.log').read_text())
r=json.loads((repeat/'result.json').read_text())
report=dict(status='second_full_resident_speed_transfer_verified',saved_ns=time.time_ns(),first=old,repeat=new,
 total_delta_s=new['elapsed_s']-old['elapsed_s'],
 group_delta_s={k:new['seconds_by_group'][k]-old['seconds_by_group'][k] for k in new['seconds_by_group']},
 first_model=prior_model,repeat_model=current_model,
 both_models_resident_reused=prior_model.get('resident_reused') is True and current_model.get('resident_reused') is True,
 same_model_pid=prior_model.get('model_pid')==current_model.get('model_pid'),
 physical_transfer_verified=r['physical_transfer_verified'],uninterrupted_transfer_verified=r.get('uninterrupted_transfer_verified'),
 current_object_location='square_low_table#living_room',final_robot_state=r.get('last_fresh_robot_state'),
 robot_ready_for_next_task=r.get('robot_ready_for_next_task'),navigation_profile=r.get('navigation_motion_profile'),
 restore_attempt_excluded=str(base/'penguin-resident-repeat-20261026-01'),
 comparison_limit='Both tasks were already warm. Total-time differences measure live execution variability, not the cold-start saving. Initial poses and source observations are reported separately.')
name='resident-model-second-timing-comparison-20261026.json'
for p in (base/name,Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training')/name):
 with p.open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:report[k] for k in ('status','total_delta_s','group_delta_s','both_models_resident_reused','same_model_pid','physical_transfer_verified','robot_ready_for_next_task')}))
