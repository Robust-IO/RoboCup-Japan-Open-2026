"""Receive-only fresh camera and idle-controller inspection before a NEW suffix trial."""
import json
import math
from pathlib import Path
import subprocess
import sys
import time

CODE=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
OUT=Path('/__HANDYMAN_PACKAGE__/runtime/apple-r19-carry-scene-check-20261022-01')
sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json
from apple_transfer_auto_plan import map_pose
from task_outcome_policy import carry_lower_from_current
import rclpy
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState

with RobotLock():
    OUT.mkdir(exist_ok=False)
    with (OUT/'capture.log').open('x') as stream:
        child=subprocess.run([sys.executable,str(CODE/'capture_grasp_scene.py'),str(OUT/'cameras'),
                              '--hand-geometry','--map-geometry','--tracking-epoch','r19-carry-scene-check',
                              '--duration','7','--interval','.6','--stop-after-frames','1'],
                             stdout=stream,stderr=subprocess.STDOUT,timeout=25)
    if child.returncode:raise ValueError('fresh_camera_probe_failed')
    last=sorted((OUT/'cameras').glob('*/snapshot.json'))[-1].parent
    snapshot=json.loads((last/'snapshot.json').read_text())
    rclpy.init();node=rclpy.create_node('r19_carry_readonly_ready')
    def call(topic,kind,request):
        client=node.create_client(kind,topic)
        if not client.wait_for_service(timeout_sec=4):raise ValueError('idle_controller_service_missing')
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=4)
        if not future.done():raise ValueError('idle_controller_read_timeout')
        return future.result()
    try:
        req=GetParameters.Request()
        req.names=['general_goal_checker.xy_goal_tolerance','general_goal_checker.yaw_goal_tolerance','FollowPath.xy_goal_tolerance']
        tolerances=[v.double_value for v in call('/controller_server/get_parameters',GetParameters,req).values]
        state=call('/controller_server/get_state',GetState,GetState.Request()).current_state.id
        end=time.monotonic()+.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        topics=['/hsrb/'+s for s in ('command_velocity','omni_base_controller/command','arm_trajectory_controller/command','gripper_controller/command','head_trajectory_controller/command')]
        publishers={t:node.count_publishers(t) for t in topics}
        subscribers={t:node.count_subscribers(t) for t in topics[1:]}
    finally:node.destroy_node();rclpy.shutdown()
    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    joints=[snapshot['joints'][k] for k in names]
    previous=json.loads(Path('/__HANDYMAN_PACKAGE__/runtime/automatic-apple-r18b-20261022-01/verified-held-geometry.json').read_text())
    validated=carry_lower_from_current(joints,previous['holding_opening_rad'])
    ready=(state in (1,2) and len(tolerances)==3 and all(abs(x-y)<1e-8 for x,y in zip(tolerances,[.06,.08,.06]))
           and not any(publishers.values()) and all(n==1 for n in subscribers.values()))
    report=dict(ready_for_explicit_current_carry_completion=ready,current_capture=str(last),
                base_map_pose=map_pose(snapshot),joints=snapshot['joints'],validated_folded_joints=validated,
                navigation_tolerances=tolerances,controller_state=state,command_publishers=publishers,
                command_subscribers=subscribers,prior_held_geometry=previous,
                holding_image_requires_review=True,robot_commands_sent=False,
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    write_json(OUT/'result.json',report)
    print(json.dumps(report),flush=True)
    if not ready:raise ValueError('current_idle_readiness_failed')
