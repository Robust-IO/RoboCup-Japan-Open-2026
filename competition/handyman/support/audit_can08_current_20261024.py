"""Read-only actual can/hand state after one inconclusive closure; no commands."""
import json,sys
from pathlib import Path
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
from canned_grasp_runtime import CanRuntime
import rclpy
base=Path('/__HANDYMAN_PACKAGE__/runtime');old=base/'can-auto-live-20261024-08/grasp';root=base/'can08-current-state-audit-20261024-01';root.mkdir(exist_ok=False)
report=json.loads((old/'result.json').read_text())
print('STOP',json.dumps({k:report.get(k) for k in ('status','stage','reason','approaches','reference','close','last_fresh_robot_state')}),flush=True)
print('CLOSURE',(old/'closure-contact.json').read_text() if (old/'closure-contact.json').exists() else 'missing',flush=True)
print('JOURNAL',(old/'journal.jsonl').read_text()[-12000:],flush=True)
with RobotLock():
    rclpy.init();rt=CanRuntime(root,'can08-readonly-current',lambda event:None)
    try:
        captures=[]
        for i in range(3):
            sample=rt.sample();captures.append(str(rt.save(sample,f'frame-{i:02d}')))
        value=dict(robot_commands_sent=False,captures=captures,current_state=rt.state())
        write_json(root/'result.json',value);print('CURRENT',json.dumps(value),flush=True)
    finally:rt.shutdown();rclpy.shutdown()
