"""Fresh receive-only camera burst after verified Unity connection."""
import json
from pathlib import Path
import subprocess
import sys
import time

CODE=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
OUT=Path('/__HANDYMAN_PACKAGE__/runtime/canned-juice-connected-20261023-01')
OUT.mkdir(exist_ok=False)
with (OUT/'capture.log').open('x') as stream:
    child=subprocess.run([sys.executable,str(CODE/'capture_grasp_scene.py'),str(OUT/'cameras'),
        '--hand-geometry','--tracking-epoch','canned-connected-20261023-01',
        '--duration','10','--interval','.6','--stop-after-frames','3','--min-frame-span','1'],
        stdout=stream,stderr=subprocess.STDOUT,timeout=30)
report=dict(robot_commands_sent=False,capture_exit_code=child.returncode,
    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),checked_ns=time.time_ns())
frames=sorted((OUT/'cameras').glob('*/snapshot.json'))
report['captures']=[str(f.parent) for f in frames]
if child.returncode==0 and frames:
    snapshot=json.loads(frames[-1].read_text())
    report.update(status='fresh_scene_received',localization_verified=False,joints=snapshot['joints'],streams=snapshot['streams'])
else:
    report.update(status='capture_incomplete',capture_tail=(OUT/'capture.log').read_text(errors='replace')[-3000:])
with (OUT/'result.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report),flush=True)
