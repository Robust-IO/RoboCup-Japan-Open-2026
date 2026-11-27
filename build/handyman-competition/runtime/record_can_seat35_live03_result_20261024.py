"""Save this completed experiment and the actual held-state provenance."""
import json
import math
import sys
import time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=base/'can-extra-seat35-live-20261024-03'
read=lambda p:json.loads(p.read_text())
result=read(root/'result.json');grasp=read(root/'grasp/result.json')
pose=result['initial_base_map_pose'];postclose=grasp['postclose_pose_observations']
value=dict(status='extra_35mm_grasp_lift_verified_tilt_not_improved',saved_ns=time.time_ns(),
    run_root=str(root),class_name='canned_juice',source='LayoutA bedroom white_side_table',
    supersedes_robot_state_record='canned-upright-calibrated-live-progress-20261024.json',
    unity_restarted_after_prior_attempt=True,old_held_context_not_reused=True,
    extra_seat_m=.035,extra_seat_default_m=0.,experiment_not_promoted_as_improvement=True,
    initial_base_map_pose=pose,initial_position_error_m=math.hypot(pose[0]-1.184,pose[1]),
    navigation_verified=all(s in result['confirmed_stages'] for s in ('preflight','navigate_source','head_view')),
    grasp_success_verified=result['grasp_success_verified'],upright_grasp_verified=result['upright_grasp_verified'],
    preclose_tilt_deg=result['preclose_tilt_deg'],postclose_before_lift=postclose,
    afterlift_tilt_degrees=result['afterlift_tilt_degrees'],holding_opening_rad=result['holding_opening_rad'],
    observed_lift_m=[r['object_delta_odom_m'][2] for r in grasp['lift_verification']],
    object_palm_follow_error_m=[r['object_palm_follow_error_m'] for r in grasp['lift_verification']],
    new_close_commands=1,lift_commands=1,release_commands=0,held_context=result['held_context'],
    robot_state_at_verified_trial_end=result['last_fresh_robot_state'],can_remained_held_at_trial_end=True,
    arm_retracted=False,placement_verified=False,full_transfer_completed=False,
    navigation_parameters_changed=False,detector_changed=False,tests_passed=600,
    finding='Postclose RGBD already measured approximately 14.6 degrees before any lift; postlift was approximately 14.5 degrees. Additional seating alone did not remove tilt in this run.',
    causal_limit='The object was manually repositioned and Unity restarted, so the old 11.3 degree result is not a controlled statistical comparison.',
    preceding_attempts=[dict(root=str(base/'can-extra-seat35-live-20261024-01'),
        reason='far_local_support_not_measurable',robot_commands_sent=False),
        dict(root=str(base/'can-extra-seat35-live-20261024-02'),
        reason='adjacent_metal_plane_in_can_cap_fit',close_commands=0,followed_by_unity_crash=True)],
    prior_requested_reset=dict(root=str(base/'can-upright-putdown-retreat1m-20261024-01'),
        measured_can_bottom_gap_before_release_m=.002170369293552299,
        actual_retreat_m=.9994154479463366,reset_complete_before_user_reposition=True),
    historical_state_is_not_current_motion_authorization=True,
    next=['Audit new current RGBD and joints before any further motion; do not repeat closure while this can is held.',
        'Inspect lateral finger centering and first contact during closure; deeper seating is configurable, not established as a successful tilt fix.',
        'If releasing, measure the current tilted can bottom and actual tabletop support first; never assume the old upright bottom.',
        'Only this run held-context is eligible for a reviewed carry continuation.'],
)
# Optional source-pose replay uses saved pixels only; never a live measurement.
try:
    import cv2,numpy as np
    sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
    from canned_grasp_geometry import measure_near
    item=grasp['initial_observations'][-1];cap=Path(item['capture']);snapshot=read(cap/'snapshot.json')
    geometry=item['geometry'];rgb=cv2.cvtColor(cv2.imread(str(cap/'rgb.png')),cv2.COLOR_BGR2RGB)
    measured=measure_near(np.load(cap/'depth_m.npy'),snapshot,geometry['center_m'],geometry['footprint_radius_m'],
        geometry['height_m'],rgb=rgb,appearance=grasp['reference']['source_appearance'])
    value['offline_source_pose_replay_before_any_arm_motion']=dict(capture=str(cap),measurement=measured)
except Exception as exc:value['optional_source_pose_replay_error']=str(exc)
path=base/'canned-extra-seat35-live-progress-20261024.json'
with path.open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
print(json.dumps(dict(saved=str(path),preclose_tilt_deg=value['preclose_tilt_deg'],
    postclose_tilt_degrees=[r.get('tilt_deg') for r in postclose],afterlift_tilt_degrees=value['afterlift_tilt_degrees'],
    source_pose_tilt_deg=value.get('offline_source_pose_replay_before_any_arm_motion',{}).get('measurement',{}).get('tilt_deg'))))
