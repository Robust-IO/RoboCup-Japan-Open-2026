"""Bounded, read-only progress view; never controls or restarts a trial."""
import json
from pathlib import Path
import time

root = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/automatic-apple-r11b-20261018-01')
end = time.monotonic()+45
while True:
    state = json.loads((root/'status.json').read_text())
    events = [json.loads(line) for line in (root/'journal.jsonl').read_text().splitlines()]
    actions = [e for e in events if e['event'] == 'command_intent']
    last = actions[-1] if actions else None
    log = root/f"command-{last['sequence']:03d}.log" if last else None
    print(json.dumps(dict(status=state['status'], stage=state.get('stage'),
        confirmed=state.get('confirmed_stages'), reason=state.get('reason'),
        last_command=last['command'][1:3] if last else None,
        last_log=log.read_text(errors='replace').splitlines()[-3:] if log and log.exists() else [])),flush=True)
    if state['status'] != 'running' or time.monotonic() >= end:
        break
    time.sleep(min(10, max(.01, end-time.monotonic())))
