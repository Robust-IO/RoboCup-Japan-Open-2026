"""Run installed ROS C++ examples in domain 73, with bounded cleanup."""
import os
from pathlib import Path
import subprocess
import tempfile
import time

assert os.environ.get('ROS_DOMAIN_ID') == '73'
out=Path(tempfile.mkdtemp(prefix='ros-official-probe-'))
processes=[]
streams=[]
try:
    for name in ('talker','listener'):
        stream=(out/(name+'.log')).open('w');streams.append(stream)
        processes.append(subprocess.Popen(['/opt/ros/humble/lib/demo_nodes_cpp/'+name],stdout=stream,stderr=stream))
    time.sleep(8)
finally:
    for proc in processes:
        if proc.poll() is None: proc.terminate()
    for proc in processes:
        try: proc.wait(timeout=3)
        except subprocess.TimeoutExpired: proc.kill();proc.wait()
    for stream in streams: stream.close()
print(out)
for name in ('talker','listener'):
    print(name,(out/(name+'.log')).read_text()[-1500:])
if not all(proc.returncode in (0,-15) for proc in processes):
    raise RuntimeError('Official example failed to start; see logs')
