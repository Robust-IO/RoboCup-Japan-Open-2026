"""Install only validated files after verifying the complete pre-change hashes."""
import ast,fcntl,hashlib,json,os,shutil,sys,time
from pathlib import Path

b=Path('/__HANDYMAN_PACKAGE__/runtime');frozen=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
lease=Path('/tmp/handyman-apple-transfer-automation.lock').open('a+')
fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
validation=json.loads((b/'gate-revision-validation-latest.json').read_text())
if validation['status']!='offline_validated_not_deployed':raise ValueError('validated_candidate_required')
stage=Path(validation['root']);manifest=json.loads((stage/'package-manifest.json').read_text());backup=Path(manifest['backup'])
active={'run_cigarette_transfer_20261101.py','flat_box_grasp.py','flat_box_place.py','flat_box_carry.py','outcome_finite_adjustment.py','navigate_carry_trial.py','start_fresh_cigarette_session_20261101.py'}
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:args=[v.decode(errors='replace') for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    except (FileNotFoundError,PermissionError):continue
    if len(args)>1 and Path(args[0]).name.startswith('python') and any(Path(v).name in active for v in args[1:3]) and (proc/'stat').read_text().split()[2]!='Z':raise ValueError('active_task_no_deployment')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
previous_path=b/'gate-revision-deployment-latest.json'
previous=json.loads(previous_path.read_text()) if previous_path.exists() else {}
prior={row['target']:row for row in previous.get('files',[])} if previous.get('backup')==str(backup) and not previous.get('rollback_completed') else {}
entries=[]
for row in manifest['files']:
    source=stage/row['relative'];target=Path(row['target'])
    if sha(source)!=row['sha256']:raise ValueError('candidate_changed_after_validation:'+str(source))
    allowed={row['before_sha256']}
    if str(target) in prior:allowed.add(prior[str(target)]['sha256'])
    if sha(target) not in allowed:raise ValueError('runtime_changed_since_backup:'+str(target))
    saved=backup/row['relative']
    if sha(saved)!=row['before_sha256']:raise ValueError('original_backup_missing_or_changed:'+str(saved))
    if source.suffix=='.py':ast.parse(source.read_text())
    entries.append(dict(row,backup=str(saved)))
stamp=time.strftime('%Y%m%d-%H%M%S');transaction=backup/('install-'+stamp);transaction.mkdir()
updates={}
progress=b/'phase5-current-work-progress-20261026.json'
value=json.loads(progress.read_text())
for row in entries:
    if row['relative'].startswith('frozen/'):value['latest_expected_hashes'][Path(row['target']).name]=row['sha256']
value['gate_revision_v2']=dict(staged_source=str(stage),backup=str(backup),validated=True,live_transfer_validated=False)
updates[progress]=value
deployment=b/'outcome-policy-deployment.json';value=json.loads(deployment.read_text())
installed={v['name']:v for v in value['installed']}
for row in entries:
    if row['relative'].startswith('runtime/'):
        name=Path(row['target']).name
        installed[name]=dict(installed.get(name,{}),name=name,sha256=row['sha256'],source=str(stage/row['relative']))
value['installed']=list(installed.values());value['gate_revision_v2']=dict(backup=str(backup),candidate=str(stage),live_transfer_validated=False)
updates[deployment]=value
for target,value in updates.items():
    prior_meta=prior.get(str(target))
    if prior_meta and sha(target)!=prior_meta['sha256']:raise ValueError('deployment_metadata_changed')
    saved=Path(prior_meta['backup']) if prior_meta else transaction/target.name
    if not prior_meta:shutil.copy2(target,saved)
    data=json.dumps(value,ensure_ascii=False,indent=2).encode()
    source=transaction/(target.name+'.new');source.write_bytes(data)
    entries.append(dict(relative='metadata/'+target.name,target=str(target),sha256=sha(source),before_sha256=sha(saved),backup=str(saved),metadata_source=str(source)))
written=[]
undo={}
try:
    for row in entries:
        target=Path(row['target']);source=Path(row.get('metadata_source',str(stage/row['relative'])))
        saved_current=transaction/'undo'/row['relative'];saved_current.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():shutil.copy2(target,saved_current);undo[str(target)]=saved_current
        else:undo[str(target)]=None
        temp=target.with_name(target.name+'.gate-v2-new');shutil.copy2(source,temp);temp.replace(target);written.append(row)
    for row in entries:
        if sha(Path(row['target']))!=row['sha256']:raise ValueError('post_install_hash_mismatch')
    report=dict(status='deployed_offline_validated',version=2,backup=str(backup),candidate=str(stage),validation=validation,files=entries,robot_commands_sent=False,live_transfer_validated=False,rollback_script=str(b/'rollback_gate_revision_20261106.py'))
    receipt=b/('gate-revision-deployment-'+stamp+'.json');receipt.write_text(json.dumps(report,indent=2));(b/'gate-revision-deployment-latest.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(dict(status=report['status'],files=len(entries),receipt=str(receipt),backup=str(backup),robot_commands_sent=False)))
except BaseException:
    for row in reversed(written):
        target=Path(row['target']);saved=undo[str(target)]
        if saved is not None:shutil.copy2(saved,target)
        elif target.exists() and sha(target)==row['sha256']:target.unlink()
    raise
