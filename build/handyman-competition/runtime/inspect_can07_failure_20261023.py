import json
from pathlib import Path
p=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/can-auto-live-20261023-07/grasp')
r=json.loads((p/'result.json').read_text())
print(json.dumps({k:r.get(k) for k in ('status','stage','reason','approaches','reference','last_fresh_robot_state')},indent=2))
print('CAPTURES',[(x.name,list(y.name for y in x.glob('*.json'))) for x in (p/'captures').iterdir()])
print('JOURNAL',(p/'journal.jsonl').read_text()[-15000:])
