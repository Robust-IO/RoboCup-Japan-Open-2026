import subprocess,json,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');output=base/'pepper-final-uninterrupted-round-20261025-01'
if output.exists():raise ValueError('new_output_required')
with (base/'pepper-final-uninterrupted-round-20261025-01.console.log').open('x') as f:
 child=subprocess.Popen(['/usr/bin/python3',str(code/'pepper_regression.py'),'--run','--rounds','1','--output',str(output)],
  stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
record=dict(pid=child.pid,launched_ns=time.time_ns(),output=str(output),stdout_file=str(base/'pepper-final-uninterrupted-round-20261025-01.console.log'),independent_of_ssh_stdout=True)
with (base/'pepper-final-uninterrupted-detached-launch-20261025.json').open('x') as f:json.dump(record,f,indent=2)
print(json.dumps(record),flush=True)
