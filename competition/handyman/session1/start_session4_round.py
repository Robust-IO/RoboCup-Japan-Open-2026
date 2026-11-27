"""Explicit Session4 initial start; inspection never sends task readiness."""
import argparse,datetime,json,subprocess
from run_sessions12 import ROOT,RUNS,processes,setup,task_command,round_spec

def main():
    p=argparse.ArgumentParser();mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--inspect-container',action='store_true');mode.add_argument('--execute',action='store_true')
    p.add_argument('--from-session4-initial',action='store_true',required=True);a=p.parse_args()
    from pathlib import Path
    for pid,args in processes():
        if any(Path(v).name in ('run_live_session1.py','run_session3_search.py','run_sessions12.py','inspect_session2_container.py') for v in args):raise RuntimeError('existing_task:'+str(pid))
    cfg=round_spec(3)['config']
    if a.execute:
        from live_config import validate,require_container_execution_ready
        c=json.loads(cfg.read_text());validate(c);require_container_execution_ready(c)
    out=RUNS/('session4-'+('inspection-' if a.inspect_container else 'auto-')+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));out.mkdir()
    (RUNS/'session4-latest.txt').write_text(str(out))
    setup(3,out)
    command=([ '/usr/bin/python3',str(ROOT/'inspect_session2_container.py'),'--config',str(cfg),'--output',str(out/'inspection') ]
        if a.inspect_container else task_command(3,cfg,out))
    raise SystemExit(subprocess.run(command).returncode)
if __name__=='__main__':main()
