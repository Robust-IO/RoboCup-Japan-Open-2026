import json,sys
from pathlib import Path
sys.path.insert(0,'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime
import rclpy
root=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/can08-after-wider-open-audit-20261024-01');root.mkdir(exist_ok=False)
with RobotLock():
    rclpy.init();rt=CanRuntime(root,root.name,lambda event:None)
    try:
        sample=rt.sample();path=rt.save(sample,'current')
        r=dict(robot_commands_sent=False,capture=str(path),current_state=rt.state());write_json(root/'result.json',r);print(json.dumps(r),flush=True)
    finally:rt.shutdown();rclpy.shutdown()
project=Path('/mnt/d/handyman/handyman-unity2')
files=[]
for p in project.rglob('*.cs'):
    if 'Library' in p.parts or 'Temp' in p.parts:continue
    if any(k in p.name.lower() for k in ('grasp','gripper','handcontroller','handcommon')):files.append(str(p))
print('UNITY_GRASP_SOURCES',json.dumps(files),flush=True)
