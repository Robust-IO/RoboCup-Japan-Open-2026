import hashlib,json,subprocess,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
restore=json.loads((base/'pepper-compact-empty-arm-restore-20261026-01/result.json').read_text())
if restore['status']!='empty_open_tucked_verified':raise ValueError('empty_restore_required')
ready=json.loads((base/'pepper-compact-realignment-ready-progress-20261026.json').read_text())
for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
 for n,h in ready['latest_expected_hashes'].items():
  if hashlib.sha256((tree/n).read_bytes()).hexdigest()!=h:raise ValueError('code_changed:'+str(tree/n))
name='pepper-compact-realign-full-transfer-20261026-01';root=base/name;receipt=base/(name+'-launch.json');log=base/(name+'-console.log')
if any(p.exists() for p in (root,receipt,log)):raise ValueError('existing_run_requires_review')
args=['bash',str(code/'run_object_transfer.sh'),'--object','ground_pepper','--run','--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision','--navigation-profile','carry_fast_v1','--output',str(root)]
with log.open('xb') as out:child=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,start_new_session=True,cwd=code)
row=dict(pid=child.pid,started_ns=time.time_ns(),command=args,output=str(root),detached=True,current_pose_start_not_spawn=True)
receipt.write_text(json.dumps(row,indent=2));print(json.dumps(row))
