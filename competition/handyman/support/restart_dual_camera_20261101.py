"""Restart only the requested dual-camera RViz, without touching robot nodes."""
from pathlib import Path
import os
import signal
import subprocess
import time

config = b'/__HANDYMAN_PACKAGE__/project/training/config/live_head_hand_rgb.rviz'
owned = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        argv = (proc / 'cmdline').read_bytes().split(b'\0')
        if argv and Path(os.fsdecode(argv[0])).name == 'rviz2' and config in argv:
            owned.append(int(proc.name))
    except (FileNotFoundError, PermissionError):
        pass
for pid in owned:
    os.kill(pid, signal.SIGTERM)
deadline = time.monotonic() + 5
while time.monotonic() < deadline and any(Path(f'/proc/{pid}/cmdline').exists() and Path(f'/proc/{pid}/cmdline').read_bytes() for pid in owned):
    time.sleep(.1)
subprocess.run(['/usr/bin/python3', '/__HANDYMAN_PACKAGE__/runtime/launch_dual_camera_20261101.py'], check=True)
