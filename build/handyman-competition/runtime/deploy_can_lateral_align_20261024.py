"""Deploy can-only axis-separated positioning; no robot commands."""
import argparse,ast,hashlib,json,os,re,shutil,subprocess,sys,tarfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--sha',required=True);args=p.parse_args()
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
roots={'runtime':Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),'training':repo/'training','module':repo/'training/automation/apple_transfer_v1'}
code=roots['runtime'];prefix='can-lateral-align-v1-20261024';v=base/(prefix+'-validation')
changed=['canned_grasp_runtime.py','OBJECT_TRANSFER.txt'];added=['can_approach_plan.py','test_can_approach_plan.py'];names=changed+added
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();read=lambda p:json.loads(p.read_text())
patch=base/(prefix+'-patch.tar.gz')
if sha(patch)!=args.sha:raise ValueError('patch_hash_mismatch')
previous=read(base/'canned-current-start-continuous-success-progress-20261023-05.json')
expected=dict(previous['latest_expected_hashes'],**read(base/'can-coherent-startup-v1-20261023-deployment.json')['hashes'])
if any(sha(code/n)!=digest for n,digest in expected.items()):raise ValueError('tested_source_changed')
old={n:sha(code/n) for n in changed};allowed={(root/n).resolve() for root in roots.values() for n in names}
for root in roots.values():
    if any(sha(root/n)!=digest for n,digest in old.items()) or any((root/n).exists() for n in added):raise ValueError('unexpected_target')
frozen={label:{p.relative_to(root).as_posix():sha(p) for p in root.rglob('*') if p.is_file()
    and '__pycache__' not in p.parts and not p.name.endswith('.pyc') and p.resolve() not in allowed}
    for label,root in roots.items()}
v.mkdir(exist_ok=False);candidate=v/'candidate';shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(patch) as archive:
    if set(archive.getnames())!=set(names):raise ValueError('unexpected_patch_files')
    for member in archive.getmembers():
        if not member.isfile() or Path(member.name).name!=member.name:raise ValueError('unsafe_archive')
        (candidate/member.name).write_bytes(archive.extractfile(member).read())
for n in names:
    if n.endswith('.py'):ast.parse((candidate/n).read_text())
with (v/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),stdout=f,stderr=subprocess.STDOUT,timeout=180)
out=(v/'tests.log').read_text();print(out[-2400:],flush=True)
if proc.returncode:raise ValueError('candidate_tests_failed')
count=int(re.search(r'Ran (\d+) tests',out).group(1));sys.path.insert(0,str(candidate))
from can_approach_plan import approach_plan
from apple_transfer_automation import RobotLock
# Replay the exact saved failed first approach as planning input, not a live lease.
failed=read(base/'can-auto-live-20261023-07/grasp/result.json')['approaches'][0]
error=failed['delta'];new=approach_plan(error)
if new['phase']!='lateral_alignment' or new['delta'][0]!=0 or new['delta'][1]!=error[1]:raise ValueError('original_diagonal_not_eliminated')
straight=approach_plan([error[0],0.,0.])
if straight['delta']!=[error[0],0.,0.]:raise ValueError('forward_distance_subdivided')
with RobotLock():
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
        if any(sha(root/n)!=digest for n,digest in old.items()):raise ValueError('concurrent_source_edit')
    with tarfile.open(v/'previous-files.tar.gz','x:gz') as archive:
        for label,root in roots.items():
            for n in changed:archive.add(root/n,arcname=label+'/'+n)
    hashes={n:sha(candidate/n) for n in names}
    for root in roots.values():
        for n in names:
            pending=root/(n+'.lateral-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n)
        if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('copy_mismatch')
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
    result=dict(status='can_lateral_alignment_deployed',tests=count,hashes=hashes,previous_hashes=old,
        failed_actual_plan=failed['delta'],new_lateral_step=new,new_straight_step=straight,
        forward_distance_subdivided=False,geometry_or_freshness_thresholds_relaxed=False,
        near_lateral_correction_clears_fingers_first=True,original_navigation_and_apple_unchanged=True,
        robot_commands_sent=False,live_alignment_verified=False)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
