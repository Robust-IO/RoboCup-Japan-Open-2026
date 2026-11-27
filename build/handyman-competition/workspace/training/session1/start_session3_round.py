"""Explicit Session3 initial-start setup; does not join the two-round runner."""
import argparse,subprocess,datetime
from pathlib import Path
from run_sessions12 import setup,processes,ROOT,RUNS

def main():
 p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true',required=True);p.add_argument('--from-session3-initial',action='store_true',required=True);a=p.parse_args()
 for pid,args in processes():
  if any(Path(v).name in ('run_live_session1.py','run_sessions12.py','run_sessions123.py','run_session3_search.py') for v in args):raise RuntimeError('existing_automatic_task:'+str(pid))
 out=RUNS/('session3-search-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));out.mkdir()
 # Existing LayoutA map/localizer and empty-hand setup; no Session1 protocol.
 setup(2,out)
 result=subprocess.run(['/usr/bin/python3',str(ROOT/'run_session3_search.py'),'--config',str(ROOT/'session3-search-config.json'),'--execute','--output',str(out/'task')])
 raise SystemExit(result.returncode)
if __name__=='__main__':main()
