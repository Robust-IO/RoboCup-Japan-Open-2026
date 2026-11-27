"""Standalone sixth-round test, without enabling unverified sequence transitions."""
import argparse,datetime,json,subprocess
from pathlib import Path
from run_sessions12 import ROOT,RUNS,processes,setup,task_command,round_spec

def main():
 p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true',required=True);p.add_argument('--from-session6-initial',action='store_true',required=True);p.parse_args()
 for pid,args in processes():
  if any(Path(v).name in ('run_live_session1.py','run_session3_search.py','run_sessions12.py','inspect_session2_container.py') for v in args):raise RuntimeError('existing_task:'+str(pid))
 from live_config import validate
 validate(json.loads(round_spec(5)['config'].read_text()))
 out=RUNS/('session6-auto-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));out.mkdir()
 (RUNS/'session6-latest.txt').write_text(str(out))
 cfg=setup(5,out)
 raise SystemExit(subprocess.run(task_command(5,cfg,out)).returncode)
if __name__=='__main__':main()
