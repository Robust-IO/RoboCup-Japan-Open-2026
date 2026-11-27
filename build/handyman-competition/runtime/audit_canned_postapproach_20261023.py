"""Capture current post-approach geometry diagnostics. No robot publishers."""
import json
from pathlib import Path
import sys

CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime
from canned_grasp_geometry import measure_near,predicted_before_contact,closure_error
import rclpy

BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=BASE/'canned-grasp-postapproach-audit-20261023-01';root.mkdir(exist_ok=False)
trial=BASE/'canned-grasp-v1-live-20261023-01';reference=json.loads((trial/'locked-can-reference.json').read_text())
rclpy.init();runtime=CanRuntime(root,reference['epoch'],lambda v:print(json.dumps(v),flush=True));rows=[]
try:
    with RobotLock():
        for i in range(3):
            sample=runtime.sample();path=runtime.save(sample,f'frame-{i:02d}')
            predicted=predicted_before_contact(reference,sample['snapshot'],reference['task_id'])
            row=dict(capture=str(path),predicted_base_m=predicted.tolist())
            try:
                row['measurement']=measure_near(sample['depth'],sample['snapshot'],predicted,reference['radius_m'],reference['height_m'])
                row['closure_error_m']=closure_error(row['measurement']['center_m'],sample['snapshot']).tolist()
            except ValueError as exc:row['rejected_reason']=str(exc)
            rows.append(row)
        result=dict(rows=rows,robot_commands_sent=False,current_state=runtime.state())
        write_json(root/'result.json',result);print(json.dumps(result),flush=True)
finally:runtime.shutdown();rclpy.shutdown()
