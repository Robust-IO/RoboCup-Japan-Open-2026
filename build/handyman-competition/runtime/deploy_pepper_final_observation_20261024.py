import hashlib,json,os,re,shutil,subprocess,sys,tarfile,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1');roots=[code,repo/'training',repo/'training/automation/apple_transfer_v1']
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
previous=read(base/'ground-pepper-released-cleanup-ready-progress-20261024.json')
stopped=read(base/'pepper-native-released-cleanup-20261024-11/result.json')
if (stopped['reason']!='fresh_carry_images_required' or not stopped['hand_released']
        or stopped['release_commands_sent'] or stopped['close_commands_sent']):raise ValueError('unexpected_last_state')
for root in roots:
    if any(sha(root/n)!=d for n,d in previous['latest_expected_hashes'].items()):raise ValueError('tested_source_changed')
names=['pepper_transfer_automation.py','test_pepper_transfer.py'];old={n:sha(code/n) for n in names}
archive=base/'pepper-final-observation-20261024.tar.gz'
if sha(archive)!=sys.argv[1]:raise ValueError('patch_changed')
v=base/'pepper-final-observation-20261024-validation';v.mkdir(exist_ok=False)
candidate=v/'candidate';shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(archive) as tar:
    if set(tar.getnames())!=set(names):raise ValueError('unexpected_files')
    for member in tar.getmembers():
        if not member.isfile():raise ValueError('unsafe_archive')
        (candidate/member.name).write_bytes(tar.extractfile(member).read())
with (v/'tests.log').open('x') as f:
    result=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),stdout=f,stderr=subprocess.STDOUT,timeout=180)
log=(v/'tests.log').read_text();print(log[-1800:],flush=True)
if result.returncode:raise ValueError('regression_failed')
tests=int(re.search(r'Ran (\d+) tests',log).group(1));sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock
with RobotLock(),tarfile.open(v/'previous-files.tar.gz','x:gz') as backup:
    latest=dict(previous['latest_expected_hashes'])
    for root in roots:
        if any(sha(root/n)!=d for n,d in previous['latest_expected_hashes'].items()):raise ValueError('concurrent_edit')
    for i,root in enumerate(roots):
        for n in names:
            backup.add(root/n,arcname=str(i)+'/'+n);pending=root/(n+'.placed-proof-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n);latest[n]=sha(candidate/n)
    for root in roots:
        if any(sha(root/n)!=d for n,d in latest.items()):raise ValueError('copy_mismatch')
    checkpoint=dict(previous,status='pepper_empty_tucked_readonly_placement_verification_ready',saved_ns=time.time_ns(),
        tests_passed=tests,latest_expected_hashes=latest,last_verified_robot_state=stopped['last_fresh_robot_state'],
        last_verified_robot_state_description='EMPTY and tucked after one release and successful withdrawal/tuck; final physical placement proof pending',
        next='Read-only three-frame object-plus-table proof from the new current pose. No grasp, release, base or arm commands.',
        last_live_attempt=str(base/'pepper-native-released-cleanup-20261024-11'))
    for p in (base/'ground-pepper-final-observation-ready-progress-20261024.json',repo/'training/ground-pepper-final-observation-ready-progress-20261024.json'):
        with p.open('x') as f:json.dump(checkpoint,f,indent=2,allow_nan=False)
print(json.dumps(dict(status=checkpoint['status'],tests_passed=tests,robot_commands_sent=False)),flush=True)
