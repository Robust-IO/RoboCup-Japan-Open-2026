"""Offline tests + original-frame regression only; no ROS controller commands."""
import argparse
import json
from pathlib import Path
import subprocess

parser=argparse.ArgumentParser();parser.add_argument('--iteration',type=int,choices=[1,2],default=1)
args=parser.parse_args()
code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
out=Path(f'/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/apple-gate-pruning-r9-validation-20261018-{args.iteration:02d}')
out.mkdir(exist_ok=False)
commands=dict(tests=['/usr/bin/python3','-m','unittest','discover','-p','test_*.py'],
    replay=['/usr/bin/python3',str(code/'replay_gate_pruning.py'),'--recorded-only',
        '--source-run','/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/automatic-apple-20261018-01',
        '--output',str(out/'recorded-approach-replay')])
children={}
for name,command in commands.items():
    with (out/(name+'.log')).open('x') as log:
        children[name]=subprocess.Popen(command,cwd=code,stdin=subprocess.DEVNULL,
            stdout=log,stderr=subprocess.STDOUT)
codes={name:child.wait(timeout=180) for name,child in children.items()}
report=dict(status='offline_gate_pruning_validation_passed' if not any(codes.values()) else 'offline_validation_failed',
    exit_codes=codes,logs=str(out),robot_commands_sent=False,live_trial_started=False)
with (out/'result.json').open('x') as stream:json.dump(report,stream,indent=2)
print(json.dumps(report),flush=True)
for name in commands:
    print(name, (out/(name+'.log')).read_text(errors='replace')[-2400:],flush=True)
if any(codes.values()):raise SystemExit(1)
