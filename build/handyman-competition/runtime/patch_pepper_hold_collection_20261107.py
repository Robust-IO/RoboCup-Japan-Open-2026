"""Remove the old aggregate 8s hold-collection deadline in the isolated adapter."""
import ast,hashlib,json,shutil
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');p=b/'pepper-dynamic-height-adapter-latest.json';d=json.loads(p.read_text());r=Path(d['adapter']);target=r/'pepper_transfer_automation.py'
shutil.copy2(target,r/'pepper_transfer_automation.before-persistent-hold.py.txt')
s=target.read_text();before="                rows=[];deadline=time.monotonic()+8;last='no_current_held_pepper'\n                while len(rows)<3 and time.monotonic()<deadline:"
after="                rows=[];last='no_current_held_pepper';notice=time.monotonic()\n                while len(rows)<3:\n                    if time.monotonic()-notice>=5:\n                        record(dict(event='waiting_for_actual_hold_observations',label=label,valid_frames=len(rows),reason=last,robot_commands_sent=False))\n                        notice=time.monotonic()"
if s.count(before)!=1:raise ValueError('expected_original_hold_window_required')
s=s.replace(before,after,1);ast.parse(s);target.write_text(s)
for row in d['files']:
    if row['name']=='pepper_transfer_automation.py':row['sha256']=hashlib.sha256(target.read_bytes()).hexdigest()
p.write_text(json.dumps(d,indent=2));(r/'dynamic-height-adapter.json').write_text(json.dumps(d,indent=2));print(json.dumps(dict(status='persistent_actual_hold_collection',three_actual_frames_still_required=True,robot_commands_sent=False)))
