"""Copy the pending-code checkpoint only; never starts ROS or robot actions."""
import hashlib
from pathlib import Path

name='canned-carry-pending-progress-20261023.json'
source=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')/name;data=source.read_bytes()
repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
roots=[Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
for root in roots:
    if not root.is_dir() or (root/name).exists():raise ValueError('existing_target_or_missing_root:'+str(root))
for root in roots:
    with (root/name).open('xb') as f:f.write(data)
    if (root/name).read_bytes()!=data:raise ValueError('checkpoint_copy_mismatch')
print('Pending-code checkpoint saved to D plus three code roots; sha256='+hashlib.sha256(data).hexdigest(),flush=True)
