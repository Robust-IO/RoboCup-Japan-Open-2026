import hashlib,json,subprocess,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');name='pepper-compact-realign-open-continuation-20261026-01'
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');repo=Path('/__HANDYMAN_PACKAGE__/project')
ready=json.loads((base/'pepper-compact-realignment-ready-progress-20261026.json').read_text())
for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
 for n,h in ready['latest_expected_hashes'].items():
  if hashlib.sha256((tree/n).read_bytes()).hexdigest()!=h:raise ValueError('code_changed:'+str(tree/n))
prior=base/'pepper-frozen-cross-object-transfer-20261026-01';d=json.loads((prior/'result.json').read_text())
if d['stage']!='close_once' or d['close_commands_sent']!=0 or d['release_commands_sent']!=0:raise ValueError('open_preclose_predecessor_required')
root=base/name;receipt=base/(name+'-launch.json');log=base/(name+'-console.log')
if any(p.exists() for p in (root,receipt,log)):raise ValueError('existing_continuation_must_be_inspected')
command=['bash',str(code/'run_object_transfer.sh'),'--object','ground_pepper','--run',
 '--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision',
 '--navigation-profile','carry_fast_v1','--continue-open-pregrasp',str(prior),'--output',str(root)]
with log.open('xb') as stream:
 child=subprocess.Popen(command,cwd=code,stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
row=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(root),detached=True,
 continuation_not_uninterrupted_trial=True,old_close_commands=0,old_motion_not_replayed=True)
receipt.write_text(json.dumps(row,indent=2));print(json.dumps(row))
