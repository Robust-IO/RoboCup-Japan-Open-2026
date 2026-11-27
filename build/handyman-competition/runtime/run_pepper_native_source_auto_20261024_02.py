"""Run one fresh native-position pepper transfer, with logs on D:."""
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time

base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
name='pepper-native-source-auto-20261024-02'
root=base/name
checkpoint=json.loads((base/'ground-pepper-capture-timing-fixed-progress-20261024.json').read_text())
for copy in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
    for path,digest in checkpoint['latest_expected_hashes'].items():
        if hashlib.sha256((copy/path).read_bytes()).hexdigest()!=digest:
            raise ValueError('tested_code_changed:'+str(copy/path))
if root.exists():raise ValueError('new_run_directory_required')
preference=dict(updated_ns=time.time_ns(),
    instruction_zh='物品本来就在桌上；以后不要默认要求用户搬动物品，先从现场原位置识别并抓取。',
    default_test_uses_native_object_position=True,
    robot_operation_authorized=True)
pref=base/'manipulation-test-preferences.json'
if pref.exists():preference=dict(json.loads(pref.read_text()),**preference)
pending=pref.with_suffix('.pending');pending.write_text(json.dumps(preference,ensure_ascii=False,indent=2)+'\n');pending.replace(pref)
command=['bash',str(code/'run_object_transfer.sh'),'--object','ground_pepper','--run',
    '--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision',
    '--output',str(root)]
with (base/(name+'-launch.json')).open('x') as f:
    json.dump(dict(command=command,started_ns=time.time_ns(),native_source_position=True,
        source_reposition_requested=False,automatic_action_retry=False),f,indent=2)
def interrupted(*_):raise KeyboardInterrupt()
signal.signal(signal.SIGTERM,interrupted)
child=None
try:
    with (base/(name+'.console.log')).open('x') as stream:
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        print(json.dumps(dict(status='launched',pid=child.pid,run_root=str(root))),flush=True)
        last=None;last_print=0
        while child.poll() is None:
            try:
                result=json.loads((root/'result.json').read_text())
                progress={k:result.get(k) for k in ('status','stage','confirmed_stages','grasp_success_verified',
                    'close_commands_sent','release_commands_sent','reason')}
                if progress!=last or time.monotonic()-last_print>=20:
                    print(json.dumps(dict(event='progress',**progress)),flush=True)
                    last=progress;last_print=time.monotonic()
            except (FileNotFoundError,json.JSONDecodeError):pass
            time.sleep(1)
        print(json.dumps(dict(status='process_exited',returncode=child.returncode,run_root=str(root))),flush=True)
        if (root/'result.json').exists():
            result=json.loads((root/'result.json').read_text())
            print(json.dumps({k:v for k,v in result.items() if k not in ('source_observations','lift_verification','final_placement_rows')}),flush=True)
        else:
            print((base/(name+'.console.log')).read_text()[-4500:],flush=True)
        raise SystemExit(child.returncode)
except KeyboardInterrupt:
    if child and child.poll() is None:
        subprocess.run(['kill','-TERM','--','-'+str(child.pid)],check=False)
        try:child.wait(timeout=12)
        except subprocess.TimeoutExpired:
            subprocess.run(['kill','-KILL','--','-'+str(child.pid)],check=False)
    raise
