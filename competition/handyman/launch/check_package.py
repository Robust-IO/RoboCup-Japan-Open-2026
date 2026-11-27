from pathlib import Path
import json,hashlib,subprocess,sys
ROOT=Path(__file__).resolve().parent
from runtime_resources import dependencies
from live_config import validate
from ament_index_python.packages import get_package_share_directory
dependencies()  # Imports only. Does not open runtime resources or create nodes.
bridge=json.loads((ROOT/'runtime/clock-bridge-active.json').read_text())
binary=Path(bridge['binary'])
assert binary.is_file(), 'missing bridge executable: '+str(binary)
assert binary.stat().st_mode & 0o111, 'bridge executable permission missing'
assert hashlib.sha256(binary.read_bytes()).hexdigest()==bridge['sha256'], 'bridge SHA256 mismatch'
link=subprocess.run(['ldd',str(binary)],capture_output=True,text=True)
assert link.returncode==0 and 'not found' not in link.stdout+link.stderr, link.stdout+link.stderr
print(json.dumps(dict(clock_bridge='passed',sha256=bridge['sha256'],shared_libraries='resolved',bridge_started=False)),flush=True)
configs=[]
for n in range(1,7):
 f=ROOT/'workspace/training/session1'/('session3-search-config.json' if n==3 else f'session{n}-live-config.json')
 c=json.loads(f.read_text());validate(c)
 for key in ['environment','semantic_map','registry','model_python']:assert Path(c[key]).exists(),(n,key,c[key])
 configs.append(c)
reg=json.loads(Path(configs[0]['registry']).read_text());weights=Path(reg['weights'])
assert hashlib.sha256(weights.read_bytes()).hexdigest()==reg['sha256']
for pkg in ['handyman_msgs','handyman_ros2','handyman_rebuild_ros2']:
 path=Path(get_package_share_directory(pkg));assert path.is_relative_to(ROOT),(pkg,str(path))
print(json.dumps(dict(configs=6,model_sha256=reg['sha256'],runtime_imports='passed',ros_packages='packaged',robot_commands_sent=False)),flush=True)
r=subprocess.run([configs[0]['model_python'],'-c','import torch,ultralytics; print("model_environment_ok",torch.cuda.is_available())'],capture_output=True,text=True,timeout=60)
print(r.stdout);assert r.returncode==0,r.stderr

from check_child_entrypoints import check
check()

# Exercise the actual Unix socket length restriction without starting a model.
import tempfile,socket
with tempfile.TemporaryDirectory(prefix='hm-ipc-',dir='/tmp') as ipc:
 endpoint=str(Path(ipc)/('f'*16+'.sock'))
 with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as server:
  server.bind(endpoint)
 print(json.dumps(dict(model_ipc='unix_socket_bind_passed',socket_path_bytes=len(endpoint.encode()),robot_commands_sent=False)),flush=True)
