"""Receive-only initial camera state; no robot publishers."""
import json
from pathlib import Path
import subprocess
import sys

CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
OUT=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/apple-r18b-timed-scene-check-20261022-01')
OUT.mkdir(exist_ok=False)
with (OUT/'capture.log').open('x') as stream:
    result=subprocess.run([sys.executable,str(CODE/'capture_grasp_scene.py'),str(OUT/'cameras'),
        '--hand-geometry','--map-geometry','--tracking-epoch','r18b-timed-scene-check',
        '--duration','7','--interval','.6','--stop-after-frames','1'],stdout=stream,stderr=subprocess.STDOUT,timeout=25)
if result.returncode:raise RuntimeError('fresh_cameras_unavailable')
sys.path.insert(0,str(CODE))
from apple_transfer_auto_plan import map_pose
capture=sorted((OUT/'cameras').glob('*/snapshot.json'))[-1].parent
snapshot=json.loads((capture/'snapshot.json').read_text())
report=dict(capture=str(capture),base_map_pose=map_pose(snapshot),joints=snapshot['joints'],
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),robot_commands_sent=False)
(OUT/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
