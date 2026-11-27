import json,time
from pathlib import Path
root=Path('/__HANDYMAN_PACKAGE__/runtime/penguin-resident-repeat-20261026-01')
p=root/'status.json'
if p.exists():
 d=json.loads(p.read_text());print('REPEAT',json.dumps({k:d[k] for k in ('status','stage','reason','restore_elapsed_s','official_timer_started','elapsed_s','total_delta_s','model_before','model_after') if k in d}))
for name in ('restore-object','timed-transfer'):
 p=root/name/'result.json'
 if not p.exists():continue
 d=json.loads(p.read_text());print(name,json.dumps({k:d[k] for k in ('status','stage','reason','elapsed_s','confirmed_stages','close_commands_sent','release_commands_sent','grasp_success_verified','physical_transfer_verified','robot_ready_for_next_task') if k in d}))
 journal=p.parent/'journal.jsonl'
 if journal.exists():
  rows=[json.loads(l) for l in journal.read_text().splitlines() if l.strip()]
  print('LATEST',json.dumps([{k:r[k] for k in ('time_ns','event','stage','label','reason') if k in r} for r in rows[-3:]]))
for name in ('restore-initial-navigation.json',):
 p=root/name
 if p.exists():
  d=json.loads(p.read_text());print(name,json.dumps({k:d[k] for k in ('status','error','speed_parameters_restored') if k in d}))
print('NOW',time.time_ns())
