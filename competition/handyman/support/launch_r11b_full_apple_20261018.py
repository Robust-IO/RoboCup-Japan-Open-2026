"""Launch ONE new full r11b simulation trial after this scene's new localization."""
import json
import os
from pathlib import Path
import subprocess
import uuid

base=Path('/__HANDYMAN_PACKAGE__/runtime')
out=base/'automatic-apple-r11b-20261018-01'
log=base/'automatic-apple-r11b-20261018-01.console.log'
startup=base/'automatic-apple-r11b-20261018-01.startup.json'
if any(p.exists() for p in (out,log,startup)):raise ValueError('new_trial_output_required_no_replay')
completion=base/'apple-r11b-initial-readiness-20261018-01/completion.json'
init=json.loads(completion.read_text())
if (init.get('status')!='new_scene_initial_localization_and_idle_readiness_verified'
    or init.get('old_numeric_anchor_reused') is not False):
    raise ValueError('new_current_scene_initialization_required')
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
cfg=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
p=cfg['policy']
if p.get('placement_approach_mode')!='single' or p['max_placement_step_m'] is not None or p['edge_margin_m']!=.03 or p['edge_uncertainty_m']!=.01:
    raise ValueError('requested_r11b_policy_not_installed')
task='apple-r11b-auto-'+uuid.uuid4().hex
command=['bash',str(code/'run_apple_transfer_auto.sh'),'--run',
    '--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision',
    '--task-id',task,'--output',str(out)]
with log.open('x') as stream:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,
        start_new_session=True,env=dict(os.environ,ROS_LOG_DIR=str(base/'automatic-apple-r11b-20261018-01-roslogs')))
record=dict(pid=child.pid,command=command,task_id=task,output=str(out),console=str(log),
    new_initial_scene_localization=str(completion),single_placement_translation_cap_m=None,
    edge_margin_m=.03,edge_uncertainty_m=.01,full_new_trial=True,automatic_retry=False,
    old_journal_replayed=False,success_not_yet_known=True)
with startup.open('x') as f:json.dump(record,f,indent=2)
print(json.dumps(record),flush=True)
