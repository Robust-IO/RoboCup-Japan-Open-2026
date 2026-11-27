"""Preserve actual retraction/near-clip outcome; no ROS/motion entrypoints."""
import hashlib
from pathlib import Path

name='canned-carry-retracted-progress-20261023.json'
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');source=base/name;data=source.read_bytes()
repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
roots=[Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),repo/'training',repo/'training/automation/apple_transfer_v1']
for root in roots:
    if not root.is_dir() or (root/name).exists():raise ValueError('new_checkpoint_target_required:'+str(root))
for root in roots:
    with (root/name).open('xb') as f:f.write(data)
    if (root/name).read_bytes()!=data:raise ValueError('checkpoint_copy_mismatch')
print('Actual tucked/closed state checkpoint saved in four places; sha256='+hashlib.sha256(data).hexdigest(),flush=True)
