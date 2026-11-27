"""New images + separately fresh current feedback; no motion or retimestamping."""
import json
from pathlib import Path
import sys

CODE=Path('/__HANDYMAN_PACKAGE__/runtime/canned-carry-v1-20261023-validation-03/candidate');sys.path.insert(0,str(CODE))
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime
from canned_carry_geometry import current_held,current_images_and_feedback
import rclpy

BASE=Path('/__HANDYMAN_PACKAGE__/runtime');root=BASE/'can-before-carry-20261023-05';root.mkdir(exist_ok=False)
reference=json.loads((BASE/'canned-grasp-lift-live-progress-20261023.json').read_text())
rclpy.init();runtime=CanRuntime(root,'can-before-carry-20261023-05',lambda e:None);rows=[]
try:
    with RobotLock():
        for i in range(3):
            sample=runtime.sample(processing_reserve_ns=0);s=sample['snapshot']
            held=current_held(sample['depth'],s,reference);state=runtime.state()
            timing=current_images_and_feedback(s,state,runtime.node.get_clock().now().nanoseconds,runtime.epoch)
            path=runtime.save(sample,f'held-{i:02d}',held['measurement'])
            rows.append(dict(held,capture=str(path),timing=timing,current_feedback=state))
        report=dict(status='current_extended_can_held',rows=rows,state=runtime.state(),robot_commands_sent=False,
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        write_json(root/'result.json',report);print(json.dumps(report),flush=True)
finally:runtime.shutdown();rclpy.shutdown()
