"""Read-only progress, with terminal duration frozen at the journal stop event."""
import json
from pathlib import Path
import time

BASE=Path('/__HANDYMAN_PACKAGE__/runtime')
ROOT=BASE/'apple-r19-carry-to-placement-20261022-01'
CONTROL=BASE/'apple-r19-carry-suffix-control-20261022-01'
read=lambda p:json.loads(p.read_text())
report={}
for key in ('status','result'):
    if (ROOT/(key+'.json')).exists():report[key]=read(ROOT/(key+'.json'))
if (ROOT/'journal.jsonl').exists():
    rows=[json.loads(line) for line in (ROOT/'journal.jsonl').read_text().splitlines() if line.strip()]
    terminal=rows[-1]['event'] in ('stopped','verified_complete')
    report['elapsed_s']=((rows[-1]['time_ns'] if terminal else time.time_ns())-rows[0]['time_ns'])/1e9
    if terminal:report['journal_duration_s']=(rows[-1]['time_ns']-rows[0]['time_ns'])/1e9
    report['last_events']=rows[-3:]
    begin={};stages={}
    for row in rows:
        if row['event']=='intent':begin[row['stage']]=row['time_ns']
        elif row['event']=='confirmed':stages[row['stage']]=(row['time_ns']-begin[row['stage']])/1e9
    report['completed_stage_seconds']=stages
logs=sorted(ROOT.glob('command-*.log'))
if logs:
    report['command_log']=str(logs[-1]);report['command_tail']=logs[-1].read_text(errors='replace')[-1600:]
if (CONTROL/'automation.log').exists():report['automation_tail']=(CONTROL/'automation.log').read_text(errors='replace')[-1800:]
if (CONTROL/'startup.json').exists():
    startup=read(CONTROL/'startup.json')
    try:argv=Path('/proc',str(startup['pid']),'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    except FileNotFoundError:argv=''
    report['process_alive']='--run-reviewed-suffix' in argv
print(json.dumps(report),flush=True)
