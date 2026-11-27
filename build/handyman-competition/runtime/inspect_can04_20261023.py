import json
from pathlib import Path
import subprocess

p=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/can-auto-live-20261023-04/grasp')
r=json.loads((p/'result.json').read_text())
print('RESULT',json.dumps(r,ensure_ascii=False))
print('FILES', '\n'.join(str(x.relative_to(p)) for x in p.rglob('*') if x.is_file()))
print('JOURNAL', (p/'journal.jsonl').read_text()[-14000:])
rows=subprocess.run(['ps','-eo','pid,args'],capture_output=True,text=True).stdout.splitlines()
print('PROCESSES','\n'.join(x for x in rows if any(t in x for t in ('can_transfer','run_canned','can_auto','ros_tcp','hold_canned'))))
