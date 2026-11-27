"""Validate actual active hotfix files and refresh their deployment provenance.

No ROS initialization or control commands. Runs while the delivery navigates.
"""
import argparse,hashlib,json,os,shutil,subprocess,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');frozen=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');receipt=b/'outcome-policy-deployment.json'
p=argparse.ArgumentParser();p.add_argument('--reason',default='stop_observe_act_v1');p.add_argument('--backup',default=str(b/'backups/before-stop-observe-act-20261101-191718'));args=p.parse_args()
backup=Path(args.backup)
if not backup.is_relative_to(b/'backups') or not backup.is_dir():raise ValueError('existing_D_backup_required')
d=json.loads(receipt.read_text());stamp=time.strftime('%Y%m%d-%H%M%S');stage=Path('/mnt/d/handyman-tools')/('outcome-hotfix-validation-'+stamp)
stage.mkdir(exist_ok=False);names=[x['name'] for x in d['installed']]
for n in ('thin_box_clearance.py','test_finger_longitudinal_overlap.py','stop_observe_act.py','test_stop_observe_act.py'):
    if n not in names:names.append(n)
for name in names:shutil.copyfile(b/name,stage/name)
shutil.copytree(frozen/'config',stage/'config')
tests=list(dict.fromkeys(d['tests']+['test_finger_longitudinal_overlap','test_stop_observe_act']));env=dict(os.environ,PYTHONPATH=str(stage)+':'+str(b)+':'+str(frozen))
with (stage/'tests.log').open('x') as out:r=subprocess.run(['/usr/bin/python3','-m','unittest',*tests,'-v'],cwd=stage,env=env,stdout=out,stderr=subprocess.STDOUT)
if r.returncode:print((stage/'tests.log').read_text()[-14000:]);raise RuntimeError('hotfix_tests_failed')
with (stage/'replay.log').open('x') as out:r=subprocess.run(['/usr/bin/python3',str(stage/'replay_outcome_transfer_20261101.py'),'--output',str(stage/'historical-replay.json')],cwd=stage,env=env,stdout=out,stderr=subprocess.STDOUT)
if r.returncode:print((stage/'replay.log').read_text()[-6000:]);raise RuntimeError('hotfix_replay_failed')
original=json.loads((Path(d['backup']['root'])/'manifest.json').read_text())['files'];by_name={x['name']:x for x in d['installed']}
changes=[]
for name in names:
    digest=hashlib.sha256((b/name).read_bytes()).hexdigest()
    if name not in by_name:by_name[name]=dict(name=name,previous_archive_key='active/'+name if 'active/'+name in original else None,supplemental=False)
    if digest!=by_name[name].get('sha256'):changes.append(name)
    by_name[name]['sha256']=digest
d['installed']=list(by_name.values());d.setdefault('runtime_hotfixes',[]).append(dict(created_ns=time.time_ns(),
    backup=str(backup),validation=str(stage),changed_files=changes,
    reason=args.reason,robot_commands_sent=False))
temp=receipt.with_suffix('.tmp');temp.write_text(json.dumps(d,ensure_ascii=False,indent=2));temp.replace(receipt)
print((stage/'tests.log').read_text()[-160:]);print(json.dumps(dict(status='hotfix_tests_and_replay_passed',validation=str(stage),changed_files=changes)))
