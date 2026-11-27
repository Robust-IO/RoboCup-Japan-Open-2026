import hashlib,json,os,re,shutil,subprocess,sys,tarfile,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');roots=[code,repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
previous=read(base/'pepper-configured-transfer-ready-progress-20261024.json')
names=['navigation_feedback_gate.py','navigate_carry_trial.py','apple_transfer_auto_backend.py','pepper_tasks.py','test_navigation_feedback_gate.py']
old={n:sha(code/n) for n in names}
for root in roots:
    if any(sha(root/n)!=d for n,d in previous['latest_expected_hashes'].items()):raise ValueError('tested_source_changed')
    if any((root/n).exists() and sha(root/n)!=d for n,d in old.items()):raise ValueError('copies_differ')
archive=base/'pepper-navigation-feedback-compat-20261024.tar.gz'
if sha(archive)!=sys.argv[1]:raise ValueError('patch_hash_changed')
v=base/'pepper-navigation-feedback-compat-20261024-validation';v.mkdir(exist_ok=False)
candidate=v/'candidate';shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(archive) as tar:
    if set(tar.getnames())!=set(names):raise ValueError('unexpected_files')
    for m in tar.getmembers():
        if not m.isfile():raise ValueError('unsafe_archive')
        (candidate/m.name).write_bytes(tar.extractfile(m).read())
with (v/'tests.log').open('x') as stream:
    p=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),stdout=stream,stderr=subprocess.STDOUT,timeout=180)
log=(v/'tests.log').read_text();print(log[-1800:],flush=True)
if p.returncode:raise ValueError('regression_failed')
tests=int(re.search(r'Ran (\d+) tests',log).group(1));sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
with RobotLock(),tarfile.open(v/'previous-files.tar.gz','x:gz') as backup:
    for root in roots:
        if any(sha(root/n)!=d for n,d in previous['latest_expected_hashes'].items()):raise ValueError('concurrent_edit')
        if any((root/n).exists() and sha(root/n)!=d for n,d in old.items()):raise ValueError('concurrent_untracked_edit')
    latest=dict(previous['latest_expected_hashes'])
    for i,root in enumerate(roots):
        for n in names:
            if (root/n).exists():backup.add(root/n,arcname=str(i)+'/'+n)
            pending=root/(n+'.feedback-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n);latest[n]=sha(candidate/n)
    for root in roots:
        if any(sha(root/n)!=d for n,d in latest.items()):raise ValueError('copy_mismatch')
    result=dict(previous,status='pepper_configured_regression_navigation_feedback_compatible',saved_ns=time.time_ns(),
        tests_passed=tests,latest_expected_hashes=latest,
        navigation_feedback_supervision_changed=True,navigation_controller_settings_changed=False,
        navigation_joint_max_age_s=.85,base_scan_and_tf_max_age_s=.7,reacquisition_budget_s=.15,
        legacy_default_joint_max_age_s=.7,robot_commands_sent=False,
        last_failed_regression=str(base/'pepper-automatic-rounds-20261024-01/result.json'),
        next='Start a NEW regression root; the initial reverse-reset attempt never grasped or released an object.')
    for path in (base/'pepper-configured-feedback-ready-progress-20261024.json',repo/'training/pepper-configured-feedback-ready-progress-20261024.json'):
        with path.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(status=result['status'],tests_passed=tests,robot_commands_sent=False)),flush=True)
