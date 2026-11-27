"""Record actual transfer evidence separately from common-core acceptance."""
import json,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
source=b/'pepper-bedroom-kitchen-20261107-153843'
finish=b/'pepper-kitchen-dynamic-continue-20261107-155837'
a=json.loads((source/'result.json').read_text());z=json.loads((finish/'result.json').read_text())
verification=json.loads((finish/'place/placement-verification.json').read_text())
common=json.loads((b/'common-placement-checkpoint-20261107.json').read_text())
if not (a['grasp_success_verified'] and z['physical_transfer_verified'] and z['placement_success_verified'] and z['robot_ready_for_next_task']):
    raise ValueError('actual_verified_transfer_evidence_required')
if len(z['final_placement_rows'])!=3 or a['close_commands_sent']!=1 or z['release_commands_sent']!=1:
    raise ValueError('actual_action_counts_or_placement_frames_wrong')
report=dict(status='pepper_transfer_verified_common_base_installed',saved_ns=time.time_ns(),
    task=z['configured_task'],source_run=str(source),placement_run=str(finish),
    grasp_success_verified=True,placement_success_verified=True,physical_transfer_verified=True,
    uninterrupted_transfer_verified=False,repair_then_continuation=True,
    close_commands_total=a['close_commands_sent']+z['close_commands_sent'],
    release_commands_total=a.get('release_commands_sent',0)+z['release_commands_sent'],
    independent_actual_placement_frames=3,placement_verification_file=str(finish/'place/placement-verification.json'),
    robot_ready_at_end_of_run=z['robot_ready_for_next_task'],robot_state_at_end_of_run=z['last_fresh_robot_state'],
    robot_state_is_historical=True,common_base=common,
    new_common_base_uninterrupted_live_test_passed=False,all_30_categories_physically_tested=False,
    current_robot_task_running=False,task_finished_sent=False,
    camera_viewer='http://100.71.59.3:18766',
    next='Read current actual state before the next task. Regress shared height logic in an uninterrupted full transfer; do not assume initial pose or replay the previous close/release.')
out=b/'phase5-current-task-checkpoint-20261107.json';out.write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(dict(checkpoint=str(out),status=report['status'],uninterrupted_transfer_verified=False,
    close_commands_total=report['close_commands_total'],release_commands_total=report['release_commands_total'],new_common_base_uninterrupted_live_test_passed=False)))
