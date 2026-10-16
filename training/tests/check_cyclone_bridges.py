"""Bounded bridge smoke test. Isolated ports 19090/51001, domain 73 only.

Never starts coordinator/navigation, never connects a client to the bridges.
Default prints commands; --run starts temporary bridge processes and cleans up.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from start_rgbd_diagnostics import listening_ports


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',action='store_true');a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp':
        p.error('Run through cyclone_test_env.py in domain 73')
    commands={
        'rosbridge':['ros2','launch','rosbridge_server','rosbridge_websocket_launch.xml','port:=19090'],
        'sigverse':['ros2','run','sigverse_ros_bridge','sigverse_ros_bridge','51001']}
    print(json.dumps(dict(commands=commands,run=a.run)),flush=True)
    if not a.run:return
    for port in (19090,51001):
        with socket.socket() as sock:sock.bind(('0.0.0.0',port))
    out=Path(tempfile.mkdtemp(prefix='cyclone-bridge-smoke-'));print('OUTPUT',out,flush=True)
    procs={};streams=[];report=dict(passed=False,domain=73,ports=[19090,51001],unity_connected=False)
    try:
        for name,command in commands.items():
            stream=(out/(name+'.log')).open('w');streams.append(stream)
            procs[name]=subprocess.Popen(command,stdout=stream,stderr=stream,start_new_session=True)
        deadline=time.monotonic()+15
        ready_since=None
        while time.monotonic()<deadline:
            if any(proc.poll() is not None for proc in procs.values()):raise RuntimeError('bridge exited; inspect logs')
            ports=listening_ports(subprocess.check_output(['ss','-ltn'],text=True))
            if {19090,51001}<=ports:
                if ready_since is None:ready_since=time.monotonic()
                if time.monotonic()-ready_since>=2:
                    # Default preflight: no camera subscription or model load.
                    check=subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/'scripts/start_rgbd_diagnostics.py'),
                        '--rosbridge-port','19090','--sigverse-port','51001'],capture_output=True,text=True,timeout=10)
                    report.update(visual_preflight_exit=check.returncode,visual_preflight=check.stdout+check.stderr)
                    if check.returncode:raise RuntimeError('visual preflight failed')
                    report['passed']=True;break
            else:ready_since=None
            time.sleep(.1)
        if not report['passed']:raise RuntimeError('bridge listen timeout')
    finally:
        for proc in procs.values():
            try:os.killpg(proc.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for proc in procs.values():
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                proc.wait(timeout=3)
        for stream in streams:stream.close()
        ports=listening_ports(subprocess.check_output(['ss','-ltn'],text=True))
        report['test_ports_released']=not ({19090,51001}&ports)
        (out/'summary.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report),flush=True)
    if not report['test_ports_released']:raise RuntimeError('test ports still listening; inspect process ownership')


if __name__=='__main__':main()
