"""Read-only progress of the new r19 timed trial."""
import json
from pathlib import Path
import time

BASE = Path('/__HANDYMAN_PACKAGE__/runtime')
ROOT = BASE / 'automatic-apple-r19-20261022-01'
CONTROL = BASE / 'apple-r19-full-live-control-20261022-01'
read = lambda p: json.loads(p.read_text())
report = {}
for name in ('status', 'result'):
    if (ROOT / (name + '.json')).exists():
        report[name] = read(ROOT / (name + '.json'))
if (ROOT / 'journal.jsonl').exists():
    rows = [json.loads(line) for line in (ROOT / 'journal.jsonl').read_text().splitlines() if line.strip()]
    terminal = bool(rows and rows[-1]['event'] in ('stopped', 'verified_complete'))
    report['elapsed_s'] = ((rows[-1]['time_ns'] if terminal else time.time_ns()) - rows[0]['time_ns']) / 1e9
    report['last_events'] = rows[-3:]
    begin = {}
    stages = {}
    for row in rows:
        if row['event'] == 'intent':
            begin[row['stage']] = row['time_ns']
        elif row['event'] == 'confirmed':
            stages[row['stage']] = (row['time_ns'] - begin[row['stage']]) / 1e9
    report['completed_stage_seconds'] = stages
    if 'result' in report or terminal:
        report['journal_duration_s'] = (rows[-1]['time_ns'] - rows[0]['time_ns']) / 1e9
if (CONTROL / 'automation.log').exists():
    report['automation_tail'] = (CONTROL / 'automation.log').read_text(errors='replace')[-1600:]
logs = sorted(ROOT.glob('command-*.log'))
if logs:
    report['command_log'] = str(logs[-1])
    report['command_tail'] = logs[-1].read_text(errors='replace')[-1600:]
if (CONTROL / 'startup.json').exists():
    startup = read(CONTROL / 'startup.json')
    try:
        argv = Path('/proc', str(startup['pid']), 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
    except FileNotFoundError:
        argv = ''
    report['automation_process_alive'] = startup['task_id'] in argv
print(json.dumps(report, ensure_ascii=False), flush=True)
