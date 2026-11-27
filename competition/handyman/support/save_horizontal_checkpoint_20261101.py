import json
from pathlib import Path
import time

base=Path('/__HANDYMAN_PACKAGE__/runtime')
root=base/'horizontal-can-source-20261101-01'
prior=json.loads((base/'phase5-current-work-progress-20261026.json').read_text())
source=json.loads((root/'result.json').read_text())
geometry=json.loads((root/'horizontal-geometry-analysis.json').read_text())
report=dict(version=1,saved_ns=time.time_ns(),status='horizontal_can_real_source_observation_complete_grasp_not_started',
    object='canned_juice',source_room='bedroom',source_table='white_side_table#bedroom_1',
    planned_destination='square_low_table#living_room',
    source_observation=str(root),source_result=source,geometry=geometry,
    latest_expected_hashes=prior['latest_expected_hashes'],
    new_geometry_module='/__HANDYMAN_PACKAGE__/runtime/horizontal_can_geometry.py',
    new_geometry_module_scope='Standalone measured-depth observation candidate; not yet integrated into any motion entry.',
    local_geometry_unit_tests_passed=5,real_rgbd_geometry_frames_passed=3,
    close_commands_sent=0,release_commands_sent=0,physical_transfer_verified=False,
    last_observed_robot_state=source['last_robot_state'],
    current_state_must_be_reacquired=True,
    rviz_status='Remote WSLg/X11 display disconnects; no confirmed visible camera window.',
    desktop_launcher='/__HANDYMAN_PACKAGE__/runtime/start_dual_camera_desktop_20261101.sh',
    known_issues=[
        'Canned-juice detector confidence in these source frames is approximately 0.603; no global threshold was changed.',
        'Can axis is across the robot: its length exceeds the open finger inner span. Do not reuse the upright-can straight approach.',
        'Need choose and validate a horizontal-object grasp direction, actual grasp, lift verification and subsequent placement.',
        'RViz remote restarts failed with X11 connection broke; operator may start the prepared script in the local Windows terminal.'
    ],
    performance_configuration_changed=False,model_weights_changed=False,navigation_route_changed=False)
name='phase5-horizontal-can-progress-20261101.json'
for target in (base/name,Path('/__HANDYMAN_PACKAGE__/project/training')/name):
    with target.open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(dict(checkpoint=str(base/name),status=report['status'],robot_commands_sent=False)))
