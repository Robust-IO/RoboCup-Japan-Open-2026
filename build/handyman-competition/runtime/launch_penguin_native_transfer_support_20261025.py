"""Launch the authorized fresh penguin transfer without an SSH stdout dependency."""
import hashlib
import json
import subprocess
import time
from pathlib import Path

base = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
root = base / 'penguin-native-transfer-20261025-10'
receipt = base / 'penguin-native-transfer-20261025-10-launch.json'
log = base / 'penguin-native-transfer-20261025-10-console.log'
ready = json.loads((base / 'penguin-source-support-ready-progress-20261025.json').read_text())
code = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
repo = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
for tree in (code, repo / 'training', repo / 'training/automation/apple_transfer_v1'):
    for name, expected in ready['latest_expected_hashes'].items():
        assert hashlib.sha256((tree / name).read_bytes()).hexdigest() == expected, str(tree / name)
if root.exists() or receipt.exists() or log.exists():
    raise RuntimeError('existing_run_must_be_inspected_before_any_relaunch')
command = ['bash', str(code / 'run_object_transfer.sh'), '--object', 'toy_penguin',
           '--run', '--confirm-layout', 'LayoutA', '--confirm-localization',
           '--simulation-ignore-collision', '--output', str(root)]
with log.open('xb') as stream:
    child = subprocess.Popen(command, cwd=code, stdin=subprocess.DEVNULL,
                             stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
row = dict(pid=child.pid, started_ns=time.time_ns(), output=str(root), console=str(log),
           command=command, three_code_copies_verified=True, detached=True)
with receipt.open('x') as stream:
    json.dump(row, stream, indent=2)
print(json.dumps(row))
