"""Reviewable small deltas over the last checked three-copy installation."""
import argparse,hashlib,json,os,re,shutil,subprocess,sys,tarfile,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--tag',required=True);p.add_argument('--sha',required=True)
p.add_argument('--previous',required=True);p.add_argument('--ready',required=True);p.add_argument('--status',required=True)
p.add_argument('--files',nargs='+',required=True);a=p.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');roots=[code,repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
old=read(base/a.previous)
for root in roots:
    if any(sha(root/n)!=h for n,h in old['latest_expected_hashes'].items()):raise ValueError('tested_source_changed')
archive=base/(a.tag+'.tar.gz')
if sha(archive)!=a.sha:raise ValueError('patch_hash_changed')
v=base/(a.tag+'-validation');v.mkdir(exist_ok=False);candidate=v/'candidate'
shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(archive) as tar:
    if set(tar.getnames())!=set(a.files):raise ValueError('unexpected_delta_files')
    for member in tar.getmembers():
        if not member.isfile() or Path(member.name).is_absolute() or '..' in Path(member.name).parts:raise ValueError('unsafe_archive')
        target=candidate/member.name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(tar.extractfile(member).read())
with (v/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),stdout=f,stderr=subprocess.STDOUT,timeout=180)
log=(v/'tests.log').read_text();print(log[-1800:],flush=True)
if proc.returncode:raise ValueError('delta_regression_failed')
tests=int(re.search(r'Ran (\d+) tests',log).group(1));sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
with RobotLock(),tarfile.open(v/'previous-files.tar.gz','x:gz') as backup:
    for root in roots:
        if any(sha(root/n)!=h for n,h in old['latest_expected_hashes'].items()):raise ValueError('concurrent_edit')
    latest=dict(old['latest_expected_hashes'])
    for i,root in enumerate(roots):
        for n in a.files:
            if (root/n).exists():backup.add(root/n,arcname=str(i)+'/'+n)
            pending=root/(n+'.delta-pending');pending.parent.mkdir(parents=True,exist_ok=True)
            with pending.open('xb') as stream:stream.write((candidate/n).read_bytes())
            pending.replace(root/n);latest[n]=sha(candidate/n)
    for root in roots:
        if any(sha(root/n)!=h for n,h in latest.items()):raise ValueError('copy_mismatch')
    result=dict(old,status=a.status,saved_ns=time.time_ns(),tests_passed=tests,latest_expected_hashes=latest,
        latest_validation_directory=str(v),robot_commands_sent=False,current_robot_state_not_queried=True)
    for path in (base/a.ready,repo/'training'/a.ready):
        with path.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
print(json.dumps(dict(status=a.status,tests_passed=tests,robot_commands_sent=False)),flush=True)
