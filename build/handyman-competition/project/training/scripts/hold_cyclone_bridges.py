"""Foreground bridge supervisor, no coordinator or robot-control nodes.

Run only through cyclone_test_env.py --domain 71 --run. Keep terminal open.
Does not stop existing processes; occupied ports cause a safe failure.
"""
import json
import argparse
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
from start_rgbd_diagnostics import listening_ports


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sigverse-executable',type=Path)
    parser.add_argument('--image-audit',action='store_true')
    args=parser.parse_args()
    if args.sigverse_executable and (not args.sigverse_executable.is_file() or not os.access(args.sigverse_executable,os.X_OK)):
        parser.error('SIGVerse executable is missing or not executable')
    if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_cyclonedds_cpp':
        raise RuntimeError('Requires explicit Cyclone environment in domain 71')
    for port in (9090,50001):
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            sock.bind(('0.0.0.0',port))
    root=Path.home()/'handyman-datasets/bridge-logs';root.mkdir(parents=True,exist_ok=True)
    logs=Path(tempfile.mkdtemp(prefix='cyclone-',dir=root))
    commands={
        'rosbridge':['ros2','launch','rosbridge_server','rosbridge_websocket_launch.xml','port:=9090'],
        'sigverse':['ros2','run','sigverse_ros_bridge','sigverse_ros_bridge','50001']}
    if args.sigverse_executable:commands['sigverse']=[str(args.sigverse_executable.resolve()),'50001']
    procs={};streams=[];stop=False
    def stopping(signum,frame):
        nonlocal stop
        stop=True
    signal.signal(signal.SIGINT,stopping);signal.signal(signal.SIGTERM,stopping)
    print('LOGS '+str(logs),flush=True)
    try:
        for name,command in commands.items():
            stream=(logs/(name+'.log')).open('w');streams.append(stream)
            env=dict(os.environ)
            if name=='sigverse':env['HANDYMAN_IMAGE_AUDIT']='1' if args.image_audit else '0'
            procs[name]=subprocess.Popen(command,env=env,stdout=stream,stderr=stream,start_new_session=True)
        deadline=time.monotonic()+20;ready=False
        while not stop:
            if any(p.poll() is not None for p in procs.values()):raise RuntimeError('Bridge exited; see '+str(logs))
            if not ready:
                ports=listening_ports(subprocess.check_output(['ss','-ltn'],text=True))
                if {9090,50001}<=ports:
                    ready=True
                    report=dict(ready=True,domain=71,rmw='rmw_cyclonedds_cpp',
                        supervisor_pid=os.getpid(),pids={k:p.pid for k,p in procs.items()},
                        unity_data_verified=False,robot_control_started=False,logs=str(logs),
                        sigverse_command=commands['sigverse'],image_audit=args.image_audit)
                    (logs/'startup.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
                elif time.monotonic()>deadline:raise RuntimeError('Bridge startup timeout')
            time.sleep(.2)
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
        print('Bridge supervisor stopped; logs preserved',flush=True)


if __name__=='__main__':main()
