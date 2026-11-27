"""Stage, test/replay, verify backup, then atomically install D-drive policy."""
import argparse,fcntl,hashlib,json,os,shutil,subprocess,time,zipfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);args=p.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime');frozen=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
backup=json.loads((base/'outcome-policy-backup-latest.json').read_text());backup_root=Path(backup['root'])
manifest=json.loads((backup_root/'manifest.json').read_text());stamp=time.strftime('%Y%m%d-%H%M%S')
prior_path=base/'outcome-policy-deployment.json'
prior=json.loads(prior_path.read_text()) if prior_path.exists() else None
prior_files={x['name']:x for x in prior['installed']} if prior and prior.get('status')=='deployed_after_validation' else {}
stage=Path('/mnt/d/handyman-tools')/('outcome-policy-stage-'+stamp);stage.mkdir(exist_ok=False)
lock=Path('/tmp/handyman-apple-transfer-automation.lock').open('a+')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
with zipfile.ZipFile(args.bundle) as z:
    for info in z.infolist():
        if Path(info.filename).name!=info.filename:raise ValueError('flat_bundle_names_required')
    z.extractall(stage)
bundle=json.loads((stage/'bundle-manifest.json').read_text())
shutil.copytree(frozen/'config',stage/'config')
for name,digest in bundle['files'].items():
    if hashlib.sha256((stage/name).read_bytes()).hexdigest()!=digest:raise ValueError('bundle_hash_mismatch:'+name)
validation=base/'outcome-policy-validation'/stamp;validation.mkdir(parents=True,exist_ok=False)
env=dict(os.environ,PYTHONPATH=str(stage)+':'+str(base)+':'+str(frozen))
commands=[['/usr/bin/python3','-m','py_compile',*[str(stage/n) for n in bundle['files'] if n.endswith('.py')]],
          ['/usr/bin/python3','-m','unittest',*bundle['tests'],'-v'],
          ['/usr/bin/python3',str(stage/'replay_outcome_transfer_20261101.py'),'--output',str(validation/'historical-replay.json')]]
for index,command in enumerate(commands):
    with (validation/f'check-{index}.log').open('x') as log:
        child=subprocess.run(command,cwd=stage,env=env,stdout=log,stderr=subprocess.STDOUT)
    if child.returncode:
        print((validation/f'check-{index}.log').read_text()[-16000:]);raise RuntimeError('candidate_validation_failed:'+str(index))
    print(json.dumps(dict(check=index,status='passed',log=str(validation/f'check-{index}.log'))),flush=True)
# Abort if another edit appeared since the verified snapshot.
for name in bundle['files']:
    old=manifest['files'].get('active/'+name);target=base/name
    expected=prior_files[name]['sha256'] if name in prior_files else old['sha256'] if old else None
    if expected and hashlib.sha256(target.read_bytes()).hexdigest()!=expected:raise ValueError('active_file_changed_since_backup_or_deployment:'+name)
if prior_files:
    with zipfile.ZipFile(backup_root/('before-outcome-update-'+stamp+'.zip'),'x') as z:
        for name in prior_files:
            if (base/name).exists():z.write(base/name,name)
extra={n:(base/n).read_bytes() for n in bundle['files'] if (base/n).exists() and 'active/'+n not in manifest['files'] and n not in prior_files}
if extra:
    with zipfile.ZipFile(backup_root/'supplemental-before-activation.zip','x') as z:
        for name,data in extra.items():z.writestr(name,data)
with zipfile.ZipFile(backup['archive']) as z:
    if z.testzip() is not None:raise ValueError('backup_crc_failure')
installed=[]
for name,digest in bundle['files'].items():
    target=base/name;temporary=base/('.outcome-install-'+name)
    shutil.copyfile(stage/name,temporary);temporary.replace(target)
    if hashlib.sha256(target.read_bytes()).hexdigest()!=digest:raise ValueError('installed_hash_mismatch:'+name)
    installed.append(dict(name=name,sha256=digest,previous_archive_key='active/'+name if 'active/'+name in manifest['files'] else None,supplemental=name in extra or prior_files.get(name,{}).get('supplemental',False)))
report=dict(status='deployed_after_validation',robot_commands_sent=False,live_validation=False,created_ns=time.time_ns(),
    backup=backup,stage=str(stage),validation=str(validation),installed=installed,tests=bundle['tests'],
    changed_navigation_parameters=False,model_changed=False,unity_files_changed=False)
(base/'outcome-policy-deployment.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(dict(status=report['status'],files=len(installed),validation=str(validation),backup=backup['root'],robot_commands_sent=False)))
