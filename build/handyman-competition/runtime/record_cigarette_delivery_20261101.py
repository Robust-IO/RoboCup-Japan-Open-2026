"""Record only independently verified completion of the current delivery."""
import json,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
def read(name):return json.loads((b/name/'result.json').read_text())
grasp=read('cigarette-contact-20261101-02');carry=read('cigarette-carry-20261101-03');place=read('cigarette-place-20261101-05')
if not (grasp.get('grasp_success_verified') and carry.get('carry_destination_verified')
        and place.get('placement_success_verified') and place.get('robot_ready_for_next_task')
        and grasp['close_commands_sent']==1 and place['release_commands_sent']==1):
    raise ValueError('actual_grasp_carry_placement_and_empty_robot_verification_required')
nav=json.loads((b/'cigarette-carry-20261101-03/destination-navigation.json').read_text())
summary=dict(schema='handyman-verified-delivery-v1',recorded_ns=time.time_ns(),status='completed',
    request='卧室里white_table上的香烟→客厅square_low_table',
    source=dict(room='bedroom',furniture='white_side_table#bedroom_1',identity='cigarette',model_detection_used=True),
    destination=dict(room='living_room',furniture='square_low_table#living_room'),
    runs=dict(source_navigation='cigarette-source-20261101-02',open_alignment='cigarette-grasp-20261101-08',
        grasp='cigarette-contact-20261101-02',carry='cigarette-carry-20261101-03',place='cigarette-place-20261101-05'),
    close_commands=1,release_commands=1,grasp_verified_by='three_actual_visible_RGBD_surface_lift_frames',
    near_contact_xy='last_full_RGBD_pose_transformed_by_actual_base_TF_with_current_visible_depth_check',
    grasp_center_xy_not_fully_remeasured_during_occlusion=True,
    placement_verification=place['placement_verification'],
    table_no_contact_policy='24_exported_robot_physical_colliders_full_opening_sweep_and_measured_table',
    robot_ready_for_next_task=True,final_state=place['final_state'],
    camera_viewer='http://127.0.0.1:8766',navigation_profile='carry_fast_v1',
    nav_reset=nav.get('nav_reset'),speed_parameters_restored=nav.get('speed_parameters_restored'),
    model_changed=False,competition_task_finished_sent=False,
    single_uninterrupted_run=False,development_pauses_and_stage_launches_occurred=True,
    placement_elapsed_s=place['elapsed_s'],carry_elapsed_s=carry['elapsed_s'],contact_and_lift_elapsed_s=grasp['elapsed_s'])
path=b/'cigarette-bedroom-white-table-to-living-square-low-table-20261101-completed.json'
with path.open('x') as stream:json.dump(summary,stream,ensure_ascii=False,indent=2)
print(json.dumps(dict(status='completed',evidence=str(path),placement_verification=place['placement_verification']),ensure_ascii=False))
