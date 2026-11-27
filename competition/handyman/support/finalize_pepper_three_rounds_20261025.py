import argparse,json,hashlib,sys,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
root=base/a.root
if root.parent!=base:raise ValueError('direct_D_drive_run_directory_required')
sys.path.insert(0,str(code))
from pepper_regression import verify_leg
from apple_transfer_automation import RobotLock
with RobotLock():
 ready=json.loads((base/'pepper-source-table-support-ready-progress-20261025.json').read_text())
 for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
  for name,want in ready['latest_expected_hashes'].items():
   if hashlib.sha256((tree/name).read_bytes()).hexdigest()!=want:raise ValueError('deployed_source_changed:'+name)
 report=json.loads((root/'result.json').read_text());sequence=json.loads((root/'sequence.json').read_text())
 if report.get('status')!='three_forward_rounds_verified' or report.get('passed_rounds')!=3 or len(report['legs'])!=6:raise ValueError('all_three_complete_rounds_required')
 rows=[]
 for planned,leg in zip(sequence,report['legs']):
  output=Path(leg['output']);result=json.loads((output/'result.json').read_text());verify_leg(result,planned['task'])
  placement=json.loads((output/'place/placement-verification.json').read_text())
  nav=json.loads((output/'destination-navigation.json').read_text())
  if nav['nav_reset'] is not True or nav['speed_parameters_restored'] is not True or nav['goal_precision_changed'] is not False:raise ValueError('navigation_restoration_required')
  events=[json.loads(line) for line in (output/'journal.jsonl').read_text().splitlines()];stages=[e for e in events if e.get('event')=='stage']
  timings=[dict(stage=x['stage'],elapsed_s=(y['time_ns']-x['time_ns'])/1e9) for x,y in zip(stages,stages[1:])]
  timings.append(dict(stage=stages[-1]['stage'],elapsed_s=(result['completed_ns']-stages[-1]['time_ns'])/1e9))
  rows.append(dict(leg=leg['leg'],kind=leg['kind'],round=leg['round'],elapsed_s=result['elapsed_s'],
   close_commands_sent=result['close_commands_sent'],release_commands_sent=result['release_commands_sent'],
   source=leg['source'],destination=leg['destination'],output=str(output),
   minimum_observed_table_outer_clearance_m=min(c['table_edge_safety']['outer_clearance_lower_estimate_m'] for c in placement['checks']),
   maximum_bottom_gap_m=max(abs(c['estimated_bottom_gap_m']) for c in placement['checks']),
   placement_center_spread_m=placement['center_spread_m'],navigation_feedback_recoveries=nav.get('feedback_recoveries',0),stage_times=timings))
 final=json.loads((Path(report['legs'][-1]['output'])/'result.json').read_text())
 value=dict(status='ground_pepper_three_complete_forward_rounds_verified',saved_ns=time.time_ns(),
  branch='feat/handyman-phase5-grasping',boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
  layout='LayoutA',class_name='ground_pepper',passed_forward_rounds=3,verified_automatic_reverse_resets=3,
  all_six_legs_uninterrupted_application_runs=True,manual_object_reposition_required=False,
  untouched_unity_spawn_start_verified=False,statistical_success_rate_established=False,
  robot_empty_tucked=True,robot_ready_for_next_task=True,object_remains_held=False,
  object_at_destination='square_low_table#living_room',last_verified_robot_state=final['last_fresh_robot_state'],
  last_state_is_historical_not_future_motion_permission=True,tests_passed=ready['tests_passed'],
  latest_expected_hashes=ready['latest_expected_hashes'],three_deployed_copies_verified=True,
  navigation_speed_route_goal_precision_unchanged=True,navigation_feedback_recovery_enabled=True,
  automatic_manipulation_retry=False,task_finished_sent=False,run_root=str(root),elapsed_s=report['elapsed_s'],legs=rows,
  preceding_interrupted_attempt='pepper-automatic-rounds-20261024-06/legs/01-round-1',
  preceding_split_completion='pepper-stopped-navigation-continuation-20261025-01',
  preceding_source_edge_failure='pepper-automatic-rounds-20261025-01',
  next='Add another already-present LayoutA object on the same furniture route; inspect its native current location and geometry, no manual relocation.')
 for path in (base/'ground-pepper-three-rounds-success-progress-20261025.json',repo/'training/ground-pepper-three-rounds-success-progress-20261025.json',root/'verified-summary.json'):
  with path.open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False)
 print(json.dumps({k:v for k,v in value.items() if k not in ('latest_expected_hashes','legs','last_verified_robot_state')},ensure_ascii=False))
 print(json.dumps([dict(leg=r['leg'],elapsed_s=r['elapsed_s'],navigation_feedback_recoveries=r['navigation_feedback_recoveries']) for r in rows]))
