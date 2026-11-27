import json,sys,time
from pathlib import Path
sys.path.insert(0,'/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime
from task_outcome_policy import pose_matches
import rclpy
base=Path('/__HANDYMAN_PACKAGE__/runtime');prior=json.loads((base/'can08-deeper-lift-probe-20261024-01/result.json').read_text())
root=base/'can08-deeper-held-now-20261024-01';root.mkdir(exist_ok=False)
with RobotLock():
    rclpy.init();rt=CanRuntime(root,root.name,lambda event:None)
    try:
        paths=[];last=0
        while len(paths)<3:
            sample=rt.sample();stamp=sample['snapshot']['streams']['depth']['stamp_ns']
            if stamp-last<200_000_000:continue
            paths.append(str(rt.save(sample,f'frame-{len(paths):02d}')));last=stamp
        state=rt.state()
        if not pose_matches(state['joints'],prior['last_fresh_robot_state']['joints']):raise ValueError('held_arm_or_grip_changed')
        result=dict(status='new_current_held_frames_captured',robot_commands_sent=False,captures=paths,current_state=state)
        write_json(root/'result.json',result);print(json.dumps(result),flush=True)
    finally:rt.shutdown();rclpy.shutdown()
