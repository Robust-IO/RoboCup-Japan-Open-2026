"""Read-only current robot state and open-finger midpoint audit."""
import json
import sys
from pathlib import Path
import numpy as np
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime
from hand_camera_ray import rotation
import rclpy
from rclpy.time import Time
base=Path('/__HANDYMAN_PACKAGE__/runtime');root=base/'can-centerline-stop-audit-20261024-01'
with RobotLock():
    root.mkdir(exist_ok=False);rclpy.init();runtime=CanRuntime(root,root.name,lambda e:None)
    try:
        sample=runtime.sample();state=runtime.state();capture=runtime.save(sample,'current-stopped')
        geometry=json.loads((base/'gripper-opening-065-v1.json').read_text())
        midpoint=np.mean([c['palm_ros_center'] for c in geometry['contacts']],axis=0)
        at=Time(nanoseconds=sample['snapshot']['streams']['depth']['stamp_ns'])
        tf=runtime.buf.lookup_transform('base_link','hand_palm_link',at).transform
        actual=runtime.tf_dict(tf);base_offset=rotation(actual['quaternion'])@midpoint
        result=dict(status='stopped_no_motion_current_empty_horizontal_pose',robot_commands_sent=False,
            robot_holding_can=False,actual_state=state,current_capture=str(capture),
            original_opening_geometry=geometry,palm_at_current_depth_time=actual,
            finger_midpoint_palm_m=midpoint.tolist(),finger_midpoint_base_offset_m=base_offset.tolist())
        write_json(root/'result.json',result);print(json.dumps(result))
    finally:runtime.shutdown();rclpy.shutdown()
