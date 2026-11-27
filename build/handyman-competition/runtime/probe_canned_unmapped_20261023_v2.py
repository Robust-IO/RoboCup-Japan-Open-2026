"""Receive-only current RGBD/robot state. No robot command publishers."""
import json
from pathlib import Path
import subprocess
import sys
import time

CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
OUT=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/canned-juice-unmapped-20261023-02')
OUT.mkdir(exist_ok=False)
with (OUT/'capture.log').open('x') as stream:
    child=subprocess.run([sys.executable,str(CODE/'capture_grasp_scene.py'),str(OUT/'cameras'),
        '--hand-geometry','--tracking-epoch','canned-unmapped-20261023-02',
        '--duration','7','--interval','.6','--stop-after-frames','1'],
        stdout=stream,stderr=subprocess.STDOUT,timeout=25)
report=dict(robot_commands_sent=False,capture_exit_code=child.returncode,
            wsl_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),checked_ns=time.time_ns())
frames=sorted((OUT/'cameras').glob('*/snapshot.json'))
if child.returncode==0 and frames:
    sys.path.insert(0,str(CODE))
    from apple_transfer_auto_plan import map_pose
    snapshot=json.loads(frames[-1].read_text())
    report.update(status='fresh_scene_received',capture=str(frames[-1].parent),
                  base_map_pose=None,localization_verified=False,joints=snapshot['joints'],streams=snapshot['streams'])
else:
    report.update(status='fresh_scene_unavailable',capture_tail=(OUT/'capture.log').read_text(errors='replace')[-2500:])
with (OUT/'result.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report),flush=True)
