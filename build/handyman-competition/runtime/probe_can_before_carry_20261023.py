"""New current held-can burst before retraction; no motion publishers."""
import json
from pathlib import Path
import sys
import time

CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime
from canned_grasp_geometry import fresh,measure_near
import rclpy

BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=BASE/'can-before-carry-20261023-01';root.mkdir(exist_ok=False)
reference=json.loads((BASE/'canned-grasp-lift-live-progress-20261023.json').read_text())
rclpy.init();runtime=CanRuntime(root,'can-before-carry-20261023-01',lambda e:None);rows=[]
try:
    with RobotLock():
        for i in range(3):
            sample=runtime.sample();s=sample['snapshot']
            target=[a+b for a,b in zip(s['palm_base_at_depth'],reference['held_center_minus_palm_base_m'])]
            measured=measure_near(sample['depth'],s,target,reference['can_footprint_radius_m'],reference['can_height_m'])
            fresh(s,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
            path=runtime.save(sample,f'held-{i:02d}',measured);rows.append(dict(capture=str(path),measurement=measured))
        report=dict(status='current_extended_can_held',rows=rows,state=runtime.state(),robot_commands_sent=False,
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        write_json(root/'result.json',report);print(json.dumps(report),flush=True)
finally:runtime.shutdown();rclpy.shutdown()
