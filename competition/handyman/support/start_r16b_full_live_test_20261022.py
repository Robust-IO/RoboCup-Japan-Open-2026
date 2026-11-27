"""Start one NEW reviewed full transfer; existing automation owns all stages."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
control=base/'apple-r16b-full-live-control-20261022-01'
output=base/'automatic-apple-r16b-20261022-01'
if control.exists() or output.exists():raise ValueError('new_trial_required_no_action_replay')
if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('reviewed_domain_required')
deployment=json.loads((base/'apple-outcome-r16b-20261018-deployment.json').read_text())
if any(hashlib.sha256((code/name).read_bytes()).hexdigest()!=digest for name,digest in deployment['hashes'].items()):
    raise ValueError('outcome_r16b_runtime_changed')
initial=base/'apple-r16b-live-initial-20261022-01'
laser=json.loads((initial/'initial-laser-audit.json').read_text())
warmup=json.loads((initial/'controller-initialization.json').read_text())
if warmup['status']!='idle_plugin_parameters_initialized_without_activation_or_parameter_changes' or any(x['inliers_within10cm']<.9 for x in laser['rows']):
    raise ValueError('fresh_scene_initialization_required')
control.mkdir()
command=[sys.executable,str(code/'apple_transfer_automation.py'),'--run',
    '--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision',
    '--task-id','apple-r16b-full-live-20261022-01','--output',str(output)]
with (control/'automation.log').open('x') as stream:
    child=subprocess.Popen(command,env=dict(os.environ,ROS_LOG_DIR=str(control/'ros-logs')),
        stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
result=dict(status='new_full_transfer_started',pid=child.pid,command=command,output=str(output),
    task_id='apple-r16b-full-live-20261022-01',runtime_revision='outcome-r16b',
    uninterrupted_full_test_requested=True,old_actions_replayed=False,automatic_retry=False,
    task_finished_protocol_enabled=False,initial_scene_laser_verified=True,navigation_tolerances_changed=False,
    log=str(control/'automation.log'))
(control/'startup.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
