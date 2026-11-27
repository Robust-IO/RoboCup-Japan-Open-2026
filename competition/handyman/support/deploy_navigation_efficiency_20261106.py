"""Install validated navigation changes and source mirrors transactionally."""
import ast,fcntl,hashlib,json,shutil,tarfile,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
leases=[]
for name in ('/tmp/handyman-apple-transfer-automation.lock','/tmp/handyman-cigarette-delivery-sequence.lock'):
    handle=Path(name).open('a+');fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB);leases.append(handle)
active={'run_cigarette_transfer_20261101.py','flat_box_grasp.py','flat_box_carry.py','flat_box_place.py','navigate_carry_trial.py','efficient_navigation.py','observe_grasp_object_20261101.py','start_fresh_cigarette_session_20261101.py'}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        args=[v.decode(errors='replace') for v in (proc/'cmdline').read_bytes().split(b'\0') if v];state=(proc/'stat').read_text().split()[2]
    except OSError:continue
    if state!='Z' and args and Path(args[0]).name.startswith('python') and any(Path(v).name in active for v in args[1:3]):raise ValueError('active_robot_task_no_install')
validation=json.loads((b/'navigation-efficiency-validation-latest.json').read_text())
if validation['status']!='offline_validated_not_deployed' or validation['tests']<124:raise ValueError('validated_navigation_candidate_required')
stage=Path(validation['root']);manifest=json.loads((stage/'package-manifest.json').read_text());backup=Path(manifest['backup'])
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
entries=[]
for row in manifest['files']:
    if row.get('target') is None:continue
    source=stage/row['relative'];target=Path(row['target'])
    if sha(source)!=row['sha256'] or sha(target)!=row['before_sha256']:raise ValueError('candidate_or_target_changed:'+str(target))
    if source.suffix=='.py':ast.parse(source.read_text())
    entries.append(dict(row))
stamp=time.strftime('%Y%m%d-%H%M%S');transaction=backup/('install-navigation-'+stamp);transaction.mkdir()
metadata={}
progress=b/'phase5-current-work-progress-20261026.json';value=json.loads(progress.read_text())
for row in entries:
    if row['relative'].startswith('frozen/'):
        value['latest_expected_hashes'][str(Path(row['target']).relative_to(f))]=row['sha256']
value['navigation_efficiency_v1']=dict(candidate=str(stage),backup=str(backup),validated=True,live_navigation_validated=False,full_live_transfer_validated=False)
metadata[progress]=value
outcome=b/'outcome-policy-deployment.json';value=json.loads(outcome.read_text());installed={v['name']:v for v in value['installed']}
for row in entries:
    if row['relative'].startswith('runtime/'):
        name=Path(row['target']).name;installed[name]=dict(installed.get(name,{}),name=name,sha256=row['sha256'],source=str(stage/row['relative']))
value['installed']=list(installed.values());value['navigation_efficiency_v1']=dict(candidate=str(stage),backup=str(backup),live_navigation_validated=False)
metadata[outcome]=value
for target,value in metadata.items():
    source=transaction/(target.name+'.new');source.write_text(json.dumps(value,ensure_ascii=False,indent=2))
    entries.append(dict(relative='metadata/'+target.name,target=str(target),sha256=sha(source),before_sha256=sha(target),metadata=True,source=str(source)))
for row in entries:
    target=Path(row['target']);saved=transaction/'undo'/row['relative'];saved.parent.mkdir(parents=True,exist_ok=True)
    if target.exists():shutil.copy2(target,saved)
    row['backup']=str(saved) if target.exists() else None
    if sha(target)!=row['before_sha256']:raise ValueError('target_changed_during_backup')
# An independent supplement preserves repository originals and metadata, too.
supplement=transaction.with_suffix('.tar.gz')
with tarfile.open(supplement,'w:gz') as t:t.add(transaction,arcname=transaction.name)
written=[]
try:
    for row in entries:
        source=Path(row.get('source',str(stage/row['relative'])));target=Path(row['target']);target.parent.mkdir(parents=True,exist_ok=True)
        temp=target.with_name(target.name+'.navigation-new');shutil.copy2(source,temp);temp.replace(target);written.append(row)
    for row in entries:
        if sha(Path(row['target']))!=row['sha256']:raise RuntimeError('installed_navigation_hash_mismatch')
    report=dict(status='installed_offline_validated',backup=str(backup),supplement=str(supplement),supplement_sha256=sha(supplement),candidate=str(stage),validation=validation,
        files=entries,robot_commands_sent=False,live_navigation_validated=False,full_live_transfer_validated=False)
    receipt=b/('navigation-efficiency-deployment-'+stamp+'.json');receipt.write_text(json.dumps(report,indent=2));(b/'navigation-efficiency-deployment-latest.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(status=report['status'],files=len(entries),backup=str(backup),supplement=str(supplement),receipt=str(receipt),robot_commands_sent=False)))
except BaseException:
    for row in reversed(written):
        target=Path(row['target'])
        if row['backup'] is None:
            if sha(target)==row['sha256']:target.unlink()
        else:shutil.copy2(row['backup'],target)
    raise
