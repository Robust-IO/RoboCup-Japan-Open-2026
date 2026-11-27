"""Preflight by default. --run explicitly starts read-only camera diagnostics."""
import argparse
import datetime
import os
from pathlib import Path
import subprocess
import sys
import uuid
from rgbd_localization import load_audit

def diagnostic_environment(inherited, domain=None):
    selected=int(inherited.get('ROS_DOMAIN_ID','71')) if domain is None else domain
    if not 0<=selected<=101: raise ValueError('domain must be 0..101')
    return dict(inherited,ROS_DOMAIN_ID=str(selected),ROS_LOCALHOST_ONLY='1')

def listening_ports(output):
    ports=set()
    for line in output.splitlines():
        fields=line.split()
        if len(fields)>=4 and fields[0]=='LISTEN':
            try: ports.add(int(fields[3].rsplit(':',1)[1]))
            except ValueError: pass
    return ports

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',action='store_true',help='Requires Unity running; omitted means preflight only')
    p.add_argument('--seconds',type=int,default=15)
    p.add_argument('--session-budget-seconds',type=float,help='Remaining session budget including model startup; replaces short capture duration')
    p.add_argument('--domain',type=int,help='Defaults to inherited ROS_DOMAIN_ID, otherwise 71')
    p.add_argument('--rosbridge-port',type=int,default=9090)
    p.add_argument('--sigverse-port',type=int,default=50001)
    p.add_argument('--target',choices=['apple','canned_juice','rabbit_doll','pink_cup','white_cup','filled_ketchup'],default='canned_juice')
    p.add_argument('--camera-audit',type=Path,default=Path('/mnt/c/Users/wpb15/Downloads/handyman-camera-audit.json'))
    args=p.parse_args()
    if not 1<=args.seconds<=120: p.error('seconds must be 1..120')
    if args.session_budget_seconds is not None:
        from search_timing import session_budget
        try:args.session_budget_seconds=session_budget(args.session_budget_seconds)
        except ValueError as exc:p.error(str(exc))
    if not all(1<=port<=65535 for port in (args.rosbridge_port,args.sigverse_port)) or args.rosbridge_port==args.sigverse_port:
        p.error('bridge ports must be distinct and in 1..65535')
    try: env=diagnostic_environment(os.environ,args.domain)
    except ValueError as exc: p.error(str(exc))
    repo=Path(__file__).resolve().parents[2]
    weights=repo/'training/runs/handyman_six_seg_20260912/weights/best.pt'
    pixi=Path('/home/crazylearner/.pixi/bin/pixi')
    for path in [weights,pixi,Path('/opt/ros/humble/setup.bash'),args.camera_audit]:
        if not path.is_file(): p.error('Missing file: '+str(path))
    try: load_audit(args.camera_audit)
    except (OSError,ValueError) as exc: p.error(str(exc))
    ports=listening_ports(subprocess.check_output(['ss','-ltn'],text=True))
    if not {args.rosbridge_port,args.sigverse_port}<=ports:
        p.error('Bridge ports missing. Start bridges in a persistent WSL terminal first. Keep Unity paused.')
    print(f'Preflight OK: files, reviewed audit, listening ports {args.rosbridge_port}/{args.sigverse_port}. ROS domain {env["ROS_DOMAIN_ID"]}; RMW {env.get("RMW_IMPLEMENTATION","system default")}.',flush=True)
    print('Listening ports do NOT prove Unity connected or fresh RGBD. No robot control is enabled.',flush=True)
    if not args.run:
        print('Preflight only; Unity may stay paused. No model loaded or subscriptions created.')
        return
    output=Path('/home/crazylearner/handyman-datasets/live')/('rgbd-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
    command=['/usr/bin/python3',str(Path(__file__).with_name('rgbd_diagnostic_node.py')),
        '--repo',str(repo),'--pixi',str(pixi),'--weights',str(weights),'--camera-audit',str(args.camera_audit.resolve()),
        '--output',str(output),'--seconds',str(args.seconds),'--target',args.target]
    if args.session_budget_seconds is not None:
        command+=['--session-budget-seconds',str(args.session_budget_seconds)]
    print('Output: '+str(output),flush=True)
    print('Unity must now be running. Ctrl+C stops diagnostics; bridges are not stopped.',flush=True)
    # Positional arguments avoid interpolating user paths into shell code.
    os.execvpe('/bin/bash',['bash','-c','source /opt/ros/humble/setup.bash && exec "$@"','rgbd',*command],env)

if __name__=='__main__': main()
