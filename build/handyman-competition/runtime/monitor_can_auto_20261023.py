"""Read the active unified trial logs; no ROS or robot commands."""
import json
from pathlib import Path
import sys

root=Path(sys.argv[1]) if len(sys.argv)>1 else Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/can-auto-live-20261023-01')
for name in ('','grasp','carry','place'):
    path=root/name/'result.json'
    if path.is_file():
        r=json.loads(path.read_text());keys=('status','stage','reason','confirmed_stages','grasp_success_verified',
            'retraction_verified','destination_navigation_verified','placement_success_verified','release_commands_sent',
            'hand_released','physical_transfer_verified','uninterrupted_transfer_verified','elapsed_s','last_fresh_robot_state')
        print(json.dumps(dict(part=name or 'coordinator',**{k:r[k] for k in keys if k in r})),flush=True)
logs=sorted(root.rglob('*.log'),key=lambda p:p.stat().st_mtime_ns,reverse=True)
for path in logs[:2]:
    print(json.dumps(dict(log=str(path),tail=path.read_text(errors='replace')[-2300:])),flush=True)
