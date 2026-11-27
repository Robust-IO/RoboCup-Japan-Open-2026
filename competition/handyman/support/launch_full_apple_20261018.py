"""Start exactly one new full transfer after reviewed fresh initial localization."""
import json
import os
from pathlib import Path
import subprocess

root = Path('/__HANDYMAN_PACKAGE__/runtime')
out = root/'automatic-apple-20261018-01'
log = root/'automatic-apple-20261018-01.console.log'
startup = root/'automatic-apple-20261018-01.startup.json'
if any(p.exists() for p in (out, log, startup)):
    raise ValueError('new_full_trial_output_required_no_replay')
init = json.loads((root/'apple-initial-readiness-20261018-01/controller-idle-parameter-initialization.json').read_text())
if init.get('status') != 'plugin_parameters_initialized_navigation_not_activated':
    raise ValueError('idle_controller_initialization_not_verified')
cmd = ['bash','/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/run_apple_transfer_auto.sh',
       '--run','--confirm-layout','LayoutA','--confirm-localization',
       '--simulation-ignore-collision','--output',str(out)]
with log.open('x') as stream:
    child = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=stream,
        stderr=subprocess.STDOUT, start_new_session=True,
        env=dict(os.environ, ROS_LOG_DIR=str(root/'automatic-apple-20261018-01-roslogs')))
record = dict(pid=child.pid, command=cmd, output=str(out), console=str(log),
              localization='New current initial-pose anchor, 3 fresh laser scans at ~99% endpoint inliers; old anchors not reused.',
              placement_approach_mode='single', full_new_trial=True, automatic_retry=False)
startup.write_text(json.dumps(record, indent=2));print(json.dumps(record),flush=True)
