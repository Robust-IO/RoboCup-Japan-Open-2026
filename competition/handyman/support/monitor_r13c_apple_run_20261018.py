"""45-second read-only progress window. Never sends/retries a robot action."""
import json
from pathlib import Path
import time

root=Path('/__HANDYMAN_PACKAGE__/runtime/automatic-apple-r13c-20261018-01')
end=time.monotonic()+45
while True:
    state=json.loads((root/'status.json').read_text());events=[]
    for line in (root/'journal.jsonl').read_text().splitlines():
        try:events.append(json.loads(line))
        except json.JSONDecodeError:pass
    actions=[e for e in events if e.get('event')=='command_intent'];last=actions[-1] if actions else None
    log=root/f"command-{last['sequence']:03d}.log" if last else None
    report=dict(status=state['status'],stage=state.get('stage'),confirmed=state.get('confirmed_stages'),
        reason=state.get('reason'),last_command=last['command'][1:3] if last else None,
        last_log=log.read_text(errors='replace').splitlines()[-3:] if log and log.exists() else [])
    index=root/'source-observe/latest-capture.json'
    if index.exists():
        entry=json.loads(index.read_text());capture=root/'source-observe'/entry['capture']
        try:
            obs=json.loads((capture/'observation.json').read_text());identity=obs.get('identity_tracking',{})
            report['source_frame']=capture.name;report['tracking']=identity.get('status')
            report['tracking_reason']=identity.get('reason');report['detector_ran']=obs.get('detector_ran')
            report['target_point_odom']=identity.get('measurement',{}).get('point_odom_m')
        except (OSError,json.JSONDecodeError):pass
    print(json.dumps(report),flush=True)
    if state['status']!='running' or time.monotonic()>=end:break
    time.sleep(min(10,max(.01,end-time.monotonic())))
