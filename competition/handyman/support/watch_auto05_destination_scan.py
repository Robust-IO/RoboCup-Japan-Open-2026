"""Passive laser audit triggered by automatic arrival; never controls robot."""
import json
from pathlib import Path
import subprocess
import sys
import time

run=Path('/__HANDYMAN_PACKAGE__/runtime/automatic-apple-20261017-05')
output=Path('/__HANDYMAN_PACKAGE__/runtime/automatic-apple-20261017-05-destination-localization-audit')
output.mkdir(exist_ok=False)
while True:
    try:status=json.loads((run/'status.json').read_text())
    except FileNotFoundError:
        time.sleep(.2);continue
    if status.get('stage') in ('observe_surface','plan_placement','prepare_placement') and status.get('status')=='running':
        with (output/'trigger.json').open('x') as f:
            json.dump(dict(status_at_trigger=status,time_ns=time.time_ns(),robot_commands_sent=False),f,indent=2)
        collector=Path('/__HANDYMAN_PACKAGE__/runtime/source-table-boundary-audit-20261017-01/capture_table_localization_audit.py')
        process=subprocess.run([sys.executable,str(collector),'--output',str(output/'scans.json')])
        print(json.dumps(dict(passive_audit='captured' if process.returncode==0 else 'capture_failed',
                              robot_commands_sent=False,returncode=process.returncode)),flush=True)
        raise SystemExit(process.returncode)
    if status.get('status')!='running':
        print(json.dumps(dict(passive_audit='skipped_run_stopped_before_destination',status=status,
                              robot_commands_sent=False)),flush=True)
        break
    if status.get('stage') in ('approach_placement','check_release','release','withdraw','verify_placement'):
        raise RuntimeError('passive_audit_trigger_missed_no_control_action_taken')
    time.sleep(.1)
