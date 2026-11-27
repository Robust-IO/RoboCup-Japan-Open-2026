from pathlib import Path
import subprocess,json,time
b=Path('/__HANDYMAN_PACKAGE__/runtime');p=b/'hold_cigarette_bridges_20261101.py';s=p.read_text()
backup=b/'hold_cigarette_bridges_20261101_initial_bind.py'
with backup.open('x') as f:f.write(s)
s=s.replace('hold-camera-bridges-thin-20261101-01','hold-camera-bridges-thin-20261101-02')
old="with socket.socket() as sock:\n        sock.bind"
assert old in s
s=s.replace(old,"with socket.socket() as sock:\n        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)\n        sock.bind")
p.write_text(s)
r=json.loads((b/'cigarette-task-bridge-restart-20261101-01.json').read_text())
with (b/'cigarette-bridge-restart-20261101-02-console.log').open('x') as log:
 child=subprocess.Popen(r['command'],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
out=dict(pid=child.pid,started_ns=time.time_ns(),port_probe_reuseaddr=True,robot_commands_sent=False)
(b/'cigarette-task-bridge-restart-20261101-02.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
