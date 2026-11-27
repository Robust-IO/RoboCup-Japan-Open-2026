"""Read only this NEW timed trial. No controls, retry or resume."""
import json
from pathlib import Path
import time

BASE=Path('/__HANDYMAN_PACKAGE__/runtime')
ROOT=BASE/'automatic-apple-r17-20261022-01'
CONTROL=BASE/'apple-r17-full-live-control-20261022-01'
read=lambda p:json.loads(p.read_text())
report={}
if (ROOT/'status.json').exists():report['status']=read(ROOT/'status.json')
if (ROOT/'result.json').exists():report['result']=read(ROOT/'result.json')
if (ROOT/'journal.jsonl').exists():
    rows=[json.loads(line) for line in (ROOT/'journal.jsonl').read_text().splitlines()]
    report['elapsed_s']=(time.time_ns()-rows[0]['time_ns'])/1e9
    report['last_events']=rows[-3:]
    begin={};stages={}
    for row in rows:
        if row['event']=='intent':begin[row['stage']]=row['time_ns']
        elif row['event']=='confirmed':stages[row['stage']]=(row['time_ns']-begin[row['stage']])/1e9
    report['completed_stage_seconds']=stages
    if 'result' in report:
        report['journal_duration_s']=(rows[-1]['time_ns']-rows[0]['time_ns'])/1e9
if (CONTROL/'automation.log').exists():report['automation_tail']=(CONTROL/'automation.log').read_text(errors='replace')[-1800:]
logs=sorted(ROOT.glob('command-*.log'))
if logs:
    report['command_log']=str(logs[-1]);report['command_tail']=logs[-1].read_text(errors='replace')[-1800:]
if (CONTROL/'startup.json').exists():
    startup=read(CONTROL/'startup.json');pid=startup['pid']
    try:argv=Path('/proc',str(pid),'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    except FileNotFoundError:argv=''
    report['automation_process_alive']=startup['task_id'] in argv
print(json.dumps(report,ensure_ascii=False),flush=True)
