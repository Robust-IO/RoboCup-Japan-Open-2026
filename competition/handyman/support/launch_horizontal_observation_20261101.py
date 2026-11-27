"""Detach this finite source observation; never start duplicate outputs."""
import json
from pathlib import Path
import subprocess
import time

base = Path('/__HANDYMAN_PACKAGE__/runtime')
name = 'horizontal-can-source-20261101-01'
root = base / name
receipt = base / (name + '-launch.json')
log = base / (name + '-console.log')
if any(p.exists() for p in (root, receipt, log)):
    raise ValueError('existing_trial_must_be_reviewed')
command = ['bash', '-lc',
    'source /opt/ros/humble/setup.bash; '
    'source /__HANDYMAN_PACKAGE__/project/install/setup.bash; '
    'export HANDYMAN_RESIDENT_MODEL=1; '
    'export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/horizontal-can-source-20261101-01-roslogs; '
    'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py '
    '--domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml '
    '--run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/observe_horizontal_can_20261101.py '
    '--output /__HANDYMAN_PACKAGE__/runtime/horizontal-can-source-20261101-01']
with log.open('xb') as stream:
    child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream,
                             stderr=subprocess.STDOUT, start_new_session=True)
data = dict(pid=child.pid, started_ns=time.time_ns(), command=command, output=str(root), log=str(log))
receipt.write_text(json.dumps(data, indent=2))
print(json.dumps(data))
