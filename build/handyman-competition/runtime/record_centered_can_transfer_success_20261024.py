"""Record verified placement and supersede the previous HELD robot state."""
import json
import time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'can-centerline-baseline-depth-visible-20261024-07'
read=lambda p:json.loads(p.read_text())
continuation=read(root/'transfer-continuation/result.json')
carry=read(root/'carry/result.json');place=read(root/'place/result.json')
proof=read(root/'place/placement-verification.json')
previous=read(base/'canned-centered-upright-grasp-success-progress-20261024.json')
if not (continuation.get('physical_transfer_verified') and place.get('placement_success_verified')
        and proof.get('placement_success_verified') and place.get('release_commands_sent')==1):
    raise ValueError('actual_transfer_placement_proof_required')
value=dict(status='centered_original_depth_can_grasp_carry_place_verified_reviewed_continuation',
    saved_ns=time.time_ns(),run_root=str(root),class_name='canned_juice',
    source='LayoutA bedroom white_side_table',destination='living_room square_low_table',
    supersedes_robot_state_record='canned-centered-upright-grasp-success-progress-20261024.json',
    grasp_success_verified=True,upright_grasp_verified=True,
    carried_to_destination_verified=carry['destination_navigation_verified'],
    retraction_verified=carry['retraction_verified'],physical_transfer_verified=True,
    placement_success_verified=True,release_commands_sent=place['release_commands_sent'],
    robot_holding_can=False,hand_released=place['hand_released'],
    cleanup_completed=place['cleanup_completed'],robot_ready_for_next_task=place.get('robot_ready_for_next_task',False),
    robot_state_at_verified_completion=place['last_fresh_robot_state'],
    actual_placement_verification=proof,final_placement_rows=place['final_placement_rows'],
    final_object_tilt_deg=[r['measurement'].get('tilt_deg') for r in place['final_placement_rows']],
    placement_approaches=place.get('approaches',[]),placement_plan=str(root/'place/placement-plan.json'),
    release_geometry=place.get('release_geometry'),
    reviewed_continuation_elapsed_s=continuation['elapsed_s'],
    initial_start_uninterrupted_full_automation_verified=False,
    full_transfer_completed_by_reviewed_grasp_then_authorized_carry_place=True,
    extra_seat_m=0.,finger_midline_base_y_offset_m=previous['finger_midline_base_y_offset_m'],
    lateral_alignment_tolerance_m=previous['lateral_alignment_tolerance_m'],
    default_can_entry_uses_corrected_midline_and_zero_extra_seating=True,
    latest_expected_hashes=previous['latest_expected_hashes'],tests_passed=previous['tests_passed'],
    model_changed=False,navigation_parameters_changed=False,task_finished_sent=False,
    statistical_grasp_success_rate_established=False,historical_state_is_not_new_motion_authorization=True,
    next=['The robot is now EMPTY after actual placement. Never resume the old held-can context as current.',
        'Use fresh RGBD/TF/joints to start a new transfer; do not replay this run carry or placement.',
        'Retain true finger-midline alignment and original forward depth (zero extra seating).',
        'Next regress a few source poses, then run a new complete initial-position trial with the unified entry.',
        'Do not claim a statistically validated success rate or a completed competition protocol from this single reviewed transfer.'])
path=base/'canned-centered-transfer-success-progress-20261024.json'
with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
print(json.dumps(dict(saved=str(path),status=value['status'],placement_success_verified=True,
    robot_holding_can=False,robot_ready_for_next_task=value['robot_ready_for_next_task'],
    final_object_tilt_deg=value['final_object_tilt_deg'],
    final_captures=[r['capture'] for r in place['final_placement_rows']])))
