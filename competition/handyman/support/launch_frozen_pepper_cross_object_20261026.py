import hashlib,json,subprocess,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');name='pepper-frozen-cross-object-transfer-20261026-01'
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');repo=Path('/__HANDYMAN_PACKAGE__/project')
checkpoint=json.loads((base/'resident-model-fast-navigation-trial-progress-20261026.json').read_text())
for tree in (code,repo/'training',repo/'training/automation/apple_transfer_v1'):
 for name_,expected in checkpoint['latest_expected_hashes'].items():
  if hashlib.sha256((tree/name_).read_bytes()).hexdigest()!=expected:raise ValueError('code_changed:'+str(tree/name_))
prior=json.loads((base/'penguin-resident-fast-transfer-20261026-02/result.json').read_text())
if prior.get('physical_transfer_verified') is not True or prior.get('robot_ready_for_next_task') is not True:
 raise ValueError('successful_penguin_repeat_required_before_next_object')
root=base/name;receipt=base/(name+'-launch.json');log=base/(name+'-console.log')
if root.exists() or receipt.exists() or log.exists():raise ValueError('existing_trial_must_be_reviewed')
command=['bash',str(code/'run_object_transfer.sh'),'--object','ground_pepper','--run','--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision','--navigation-profile','carry_fast_v1','--output',str(root)]
with log.open('xb') as out:
 child=subprocess.Popen(command,cwd=code,stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
row=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(root),log=str(log),detached=True,model_resident=True,navigation_profile='carry_fast_v1')
with receipt.open('x') as stream:json.dump(row,stream,indent=2)
print(json.dumps(row))
