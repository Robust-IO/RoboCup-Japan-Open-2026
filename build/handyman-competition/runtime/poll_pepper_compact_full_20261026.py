import json,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');root=b/'pepper-compact-realign-full-transfer-20261026-01'
for name in ('result.json','source-navigation.json','destination-navigation.json'):
 p=root/name
 if not p.exists():continue
 d=json.loads(p.read_text());keys=('status','stage','reason','elapsed_s','confirmed_stages','close_commands_sent','release_commands_sent','grasp_success_verified','physical_transfer_verified','robot_ready_for_next_task','error','speed_parameters_restored','feedback_recoveries')
 row={k:d[k] for k in keys if k in d}
 if 'speed_contexts' in d:row['speed_caps_used']=sorted({tuple(v['requested_caps']) for v in d['speed_contexts']})
 if 'reached' in d:row['reached_waypoint_count']=len(d['reached'])
 print(name,json.dumps(row))
p=root/'journal.jsonl'
if p.exists():
 rows=[json.loads(l) for l in p.read_text().splitlines() if l.strip()]
 for r in rows[-5:]:print(json.dumps({k:r[k] for k in ('time_ns','event','stage','label','reason','duration_s') if k in r}))
print('NOW',time.time_ns())
