import json
from pathlib import Path
r=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/pepper-final-uninterrupted-round-20261025-01')
d=json.loads((r/'result.json').read_text());p=r/'legs'/d['current_leg'];x=json.loads((p/'result.json').read_text())
print(json.dumps(dict(overall=d['status'],leg=d['current_leg'],stage=x.get('stage'),reason=x.get('reason'),closed=x.get('close_commands_sent'),released=x.get('release_commands_sent'),passed=d['passed_rounds'],completed_legs=d['legs'])))
logs=sorted(p.glob('command-*.log'))
if logs:print('\n'.join(logs[-1].read_text(errors='replace').splitlines()[-2:])[-1000:])
