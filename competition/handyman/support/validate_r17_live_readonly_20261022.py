"""Read-only current sensor checks. Never invokes an action or creates a publisher."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

CODE=Path('/__HANDYMAN_PACKAGE__/runtime/apple-efficiency-r17-20261022-validation-02/candidate')
OUT=Path('/__HANDYMAN_PACKAGE__/runtime/apple-efficiency-r17-20261022-live-readonly-01')
OUT.mkdir(exist_ok=False)
if os.environ.get('ROS_DOMAIN_ID')!='71':raise RuntimeError('domain71_required')
results={}
for name,args in [
    ('fast-preflight',[CODE/'inspect_transfer_ready.py','--output',OUT/'ready.json','--efficient-transfer']),
    ('one-frame',[CODE/'capture_grasp_scene.py',OUT/'one-frame','--hand-geometry','--map-geometry',
                  '--tracking-epoch','r17-readonly-one','--duration','7','--interval','.6',
                  '--stop-after-frames','1','--min-frame-span','0']),
    ('three-frame',[CODE/'capture_grasp_scene.py',OUT/'three-frame','--hand-geometry','--map-geometry',
                  '--tracking-epoch','r17-readonly-three','--duration','7','--interval','.6',
                  '--stop-after-frames','3','--min-frame-span','1.2'])]:
    start=time.monotonic()
    with (OUT/(name+'.log')).open('x') as stream:
        result=subprocess.run([sys.executable,*map(str,args)],stdout=stream,stderr=subprocess.STDOUT,timeout=25)
    value=dict(exit=result.returncode,elapsed_s=time.monotonic()-start,robot_commands_sent=False)
    if name=='fast-preflight' and (OUT/'ready.json').exists():value['ready']=json.loads((OUT/'ready.json').read_text())
    if name!='fast-preflight' and (OUT/name/'manifest.json').exists():
        frames=json.loads((OUT/name/'manifest.json').read_text())['captures']
        value['frames']=len(frames)
        value['depth_span_s']=(frames[-1]['stamps_ns'][1]-frames[0]['stamps_ns'][1])/1e9 if frames else None
    results[name]=value
    print(json.dumps(dict(name=name,**value)),flush=True)
    if result.returncode:break
with (OUT/'result.json').open('x') as stream:json.dump(dict(checks=results,robot_commands_sent=False),stream,indent=2)
