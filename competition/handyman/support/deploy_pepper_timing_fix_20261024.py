import hashlib,json,os,re,shutil,subprocess,sys,tarfile,time
from pathlib import Path

base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
roots=[code,repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
previous=read(base/'ground-pepper-adapter-ready-progress-20261024.json')
stopped=read(base/'pepper-native-source-auto-20261024-01/result.json')
if (stopped['reason']!='waiting_for_coherent_image_time_tf' or stopped['close_commands_sent']
        or stopped['release_commands_sent']):raise ValueError('unexpected_stopped_state')
for root in roots:
    if any(sha(root/n)!=digest for n,digest in previous['latest_expected_hashes'].items()):
        raise ValueError('source_changed')
validation=base/'pepper-capture-timing-fix-20261024-validation';validation.mkdir(exist_ok=False)
candidate=validation/'candidate';shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
names=['pepper_runtime.py','test_pepper_transfer.py']
archive=base/'pepper-capture-timing-fix-20261024.tar.gz'
if sha(archive)!=sys.argv[1]:raise ValueError('patch_changed')
with tarfile.open(archive) as tar:
    if set(tar.getnames())!=set(names):raise ValueError('unexpected_files')
    for member in tar.getmembers():
        if not member.isfile():raise ValueError('unsafe_file')
        (candidate/member.name).write_bytes(tar.extractfile(member).read())
with (validation/'tests.log').open('x') as stream:
    process=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],
        cwd=candidate,env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),
        stdout=stream,stderr=subprocess.STDOUT,timeout=180)
log=(validation/'tests.log').read_text();print(log[-1800:],flush=True)
if process.returncode:raise ValueError('regression_failed')
tests=int(re.search(r'Ran (\d+) tests',log).group(1))
sys.path.insert(0,str(code));from apple_transfer_automation import RobotLock
with RobotLock(),tarfile.open(validation/'previous-files.tar.gz','x:gz') as backup:
    for i,root in enumerate(roots):
        for n in names:
            if sha(root/n)!=previous['latest_expected_hashes'][n]:raise ValueError('concurrent_edit')
            backup.add(root/n,arcname=str(i)+'/'+n)
    latest=dict(previous['latest_expected_hashes'])
    for n in names:
        latest[n]=sha(candidate/n)
        for root in roots:
            pending=root/(n+'.timing-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n)
    for root in roots:
        if any(sha(root/n)!=digest for n,digest in latest.items()):raise ValueError('deployment_mismatch')
    checkpoint=dict(previous,status='pepper_capture_budget_fixed_native_source_live_retry_ready',
        saved_ns=time.time_ns(),tests_passed=tests,latest_expected_hashes=latest,
        capture_processing_reserve_ns=100_000_000,consumer_freshness_gate_changed=False,
        timing_diagnosis=str(base/'pepper-native-capture-timing-20261024-01/result.json'),
        source_reposition_requested=False,
        first_live_preparation=['Use the object already on its table; do not request default repositioning.',
            'Reacquire current empty/tucked readiness and current RGBD; continue from the source table.'],
        next='Fresh full run from current bedroom source-table pose, using native pepper position.',
        last_live_attempt=str(base/'pepper-native-source-auto-20261024-01'),
        last_verified_robot_state=stopped['last_fresh_robot_state'])
    for p in [base/'ground-pepper-capture-timing-fixed-progress-20261024.json',
        repo/'training/ground-pepper-capture-timing-fixed-progress-20261024.json']:
        with p.open('x') as f:json.dump(checkpoint,f,indent=2,ensure_ascii=False,allow_nan=False)
print(json.dumps(dict(status=checkpoint['status'],tests_passed=tests,robot_commands_sent=False)),flush=True)
