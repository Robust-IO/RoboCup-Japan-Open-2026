"""Install the validated shared core and metadata with transactional rollback."""
import ast,fcntl,hashlib,json,shutil,tarfile,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
leases=[]
for name in ('/tmp/handyman-apple-transfer-automation.lock','/tmp/handyman-cigarette-delivery-sequence.lock'):
    handle=Path(name).open('a+');fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB);leases.append(handle)
active={'run_cigarette_transfer_20261101.py','flat_box_grasp.py','flat_box_carry.py','flat_box_place.py',
    'navigate_carry_trial.py','efficient_navigation.py','pepper_transfer_automation.py','run_object_transfer.py',
    'horizontal_can_place.py','continue_pepper_dynamic_height_20261107.py'}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        args=[x.decode(errors='replace') for x in (proc/'cmdline').read_bytes().split(b'\0') if x]
        state=(proc/'stat').read_text().split()[2]
    except OSError:continue
    if state!='Z' and args and Path(args[0]).name.startswith('python') and any(Path(x).name in active for x in args[1:3]):
        raise ValueError('active_robot_task_no_install')
validation=json.loads((b/'common-placement-validation-latest.json').read_text())
if validation['status']!='offline_validated_not_deployed' or validation['tests']!=149 or not Path(validation['recorded_replay']).exists():
    raise ValueError('validated_candidate_required')
stage=Path(validation['root']);manifest=json.loads((stage/'package-manifest.json').read_text())
backup=Path(manifest['backup'])
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
entries=[]
for row in manifest['files']:
    if row['target'] is None:continue
    source=stage/row['relative'];target=Path(row['target'])
    if sha(source)!=row['sha256'] or sha(target)!=row['before_sha256']:raise ValueError('candidate_or_target_changed:'+str(target))
    if source.suffix=='.py':ast.parse(source.read_text())
    entries.append(dict(row,source=str(source)))
stamp=time.strftime('%Y%m%d-%H%M%S');transaction=backup/('install-common-placement-'+stamp);transaction.mkdir()
metadata={}
progress=b/'phase5-current-work-progress-20261026.json';value=json.loads(progress.read_text())
for row in entries:
    if row['relative'].startswith('frozen/'):
        value['latest_expected_hashes'][str(Path(row['target']).relative_to(f))]=row['sha256']
value['common_table_transfer_v1']=dict(candidate=str(stage),backup=str(backup),offline_tests=149,
    recorded_high_table_replay_passed=True,full_live_transfer_validated=False,
    previous_dynamic_height_physical_evidence=str(b/'pepper-kitchen-dynamic-continue-20261107-155837'),
    physical_evidence_was_isolated_adapter=True)
metadata[progress]=value
outcome=b/'outcome-policy-deployment.json';value=json.loads(outcome.read_text())
installed={row['name']:row for row in value['installed']}
for row in entries:
    if row['relative'].startswith('runtime/'):
        name=Path(row['target']).name
        installed[name]=dict(installed.get(name,{}),name=name,sha256=row['sha256'],source=row['source'])
value['installed']=list(installed.values())
value['common_table_transfer_v1']=dict(candidate=str(stage),backup=str(backup),full_live_transfer_validated=False)
metadata[outcome]=value
for target,value in metadata.items():
    source=transaction/(target.name+'.new');source.write_text(json.dumps(value,ensure_ascii=False,indent=2))
    entries.append(dict(relative='metadata/'+target.name,target=str(target),source=str(source),
        sha256=sha(source),before_sha256=sha(target),metadata=True))
for row in entries:
    target=Path(row['target']);saved=transaction/'undo'/row['relative'];saved.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():shutil.copy2(target,saved)
    row['backup']=str(saved) if target.exists() else None
    if sha(target)!=row['before_sha256']:raise ValueError('target_changed_during_backup')
supplement=transaction.with_suffix('.tar.gz')
with tarfile.open(supplement,'w:gz') as t:t.add(transaction,arcname=transaction.name)
written=[]
try:
    for row in entries:
        target=Path(row['target']);temp=target.with_name(target.name+'.common-new')
        shutil.copy2(row['source'],temp);temp.replace(target);written.append(row)
    for row in entries:
        if sha(Path(row['target']))!=row['sha256']:raise ValueError('installed_hash_mismatch')
    report=dict(status='installed_offline_validated',candidate=str(stage),backup=str(backup),supplement=str(supplement),
        supplement_sha256=sha(supplement),files=entries,validation=validation,robot_commands_sent=False,
        full_live_transfer_validated=False,all_30_categories_physically_tested=False)
    receipt=b/('common-placement-deployment-'+stamp+'.json');receipt.write_text(json.dumps(report,indent=2))
    (b/'common-placement-deployment-latest.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(status=report['status'],receipt=str(receipt),files=len(entries),backup=str(backup),robot_commands_sent=False)))
except BaseException:
    for row in reversed(written):
        target=Path(row['target'])
        if row['backup'] is None:target.unlink(missing_ok=True)
        else:shutil.copy2(row['backup'],target)
    raise
