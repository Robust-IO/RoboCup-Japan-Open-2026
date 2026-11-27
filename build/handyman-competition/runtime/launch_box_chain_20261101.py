import subprocess,json,time,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--grasp-attempt',type=int,default=9);p.add_argument('--chain-attempt',type=int,default=1);p.add_argument('--grasp-run');args=p.parse_args()
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');name='cigarette-delivery-outcome-20261101-'+format(args.chain_attempt,'02d');log=b/(name+'-console.log')
with log.open('xb') as stream:
    p=subprocess.Popen(['/usr/bin/python3',str(b/'chain_box_delivery_20261101.py'),'--grasp-attempt',str(args.grasp_attempt),'--chain-attempt',str(args.chain_attempt)]+(['--grasp-run',args.grasp_run] if args.grasp_run else []),stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=p.pid,started_ns=time.time_ns(),log=str(log))
with (b/(name+'-launch.json')).open('x') as stream:json.dump(receipt,stream,indent=2)
print(json.dumps(receipt))
