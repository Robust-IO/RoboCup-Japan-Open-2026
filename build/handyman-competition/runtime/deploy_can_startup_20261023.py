"""Deploy opt-in current-TF startup latching; no robot commands."""
import argparse,ast,hashlib,json,os,re,shutil,subprocess,sys,tarfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--sha',required=True);args=p.parse_args()
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');repo=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project')
roots={'runtime':Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1'),'training':repo/'training','module':repo/'training/automation/apple_transfer_v1'}
code=roots['runtime'];prefix='can-coherent-startup-v1-20261023';v=base/(prefix+'-validation')
changed=['pregrasp_raised_trial.py','canned_grasp_runtime.py'];added=['can_arm_startup.py','test_can_arm_startup.py'];names=changed+added
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();read=lambda p:json.loads(p.read_text())
patch=base/(prefix+'-patch.tar.gz')
if sha(patch)!=args.sha:raise ValueError('patch_hash_mismatch')
previous=read(base/'canned-current-start-continuous-success-progress-20261023-05.json')
if any(sha(code/n)!=digest for n,digest in previous['latest_expected_hashes'].items()):raise ValueError('previous_tested_code_changed')
old={n:sha(code/n) for n in changed};allowed={(root/n).resolve() for root in roots.values() for n in names}
for root in roots.values():
    if any(sha(root/n)!=digest for n,digest in old.items()) or any((root/n).exists() for n in added):raise ValueError('unexpected_target')
frozen={label:{p.relative_to(root).as_posix():sha(p) for p in root.rglob('*') if p.is_file()
    and '__pycache__' not in p.parts and not p.name.endswith('.pyc') and p.resolve() not in allowed}
    for label,root in roots.items()}
cfg=read(code/'config/apple_transfer_auto_v1.json');registry=read(Path(cfg['registry']));model_sha=sha(Path(registry['weights']))
v.mkdir(exist_ok=False);candidate=v/'candidate';shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(patch) as archive:
    if set(archive.getnames())!=set(names):raise ValueError('unexpected_patch_files')
    for member in archive.getmembers():
        if not member.isfile() or Path(member.name).name!=member.name:raise ValueError('unsafe_archive')
        (candidate/member.name).write_bytes(archive.extractfile(member).read())
for n in names:ast.parse((candidate/n).read_text())
with (v/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),stdout=f,stderr=subprocess.STDOUT,timeout=180)
out=(v/'tests.log').read_text();print(out[-2500:],flush=True)
if proc.returncode:raise ValueError('candidate_tests_failed')
count=int(re.search(r'Ran (\d+) tests',out).group(1))
# Aside from the opt-in startup selection, verify the movement and feedback
# functions have identical ASTs. Default apple startup takes its original path.
def definitions(path):
    tree=ast.parse(path.read_text());return {n.name:ast.dump(n,include_attributes=False) for n in ast.walk(tree)
        if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef))}
before=definitions(code/'pregrasp_raised_trial.py');after=definitions(candidate/'pregrasp_raised_trial.py')
preserved=['expected_palm','cb','check_fresh','fresh','verify_tf','move','execute_steps']
if any(before[n]!=after[n] for n in preserved):raise ValueError('motion_or_feedback_function_changed')
sys.path.insert(0,str(candidate));from apple_transfer_automation import RobotLock
def frozen_unchanged():
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
    if sha(Path(registry['weights']))!=model_sha:raise ValueError('model_changed')
with RobotLock():
    frozen_unchanged()
    for root in roots.values():
        if any(sha(root/n)!=digest for n,digest in old.items()):raise ValueError('concurrent_source_edit')
    with tarfile.open(v/'previous-files.tar.gz','x:gz') as archive:
        for label,root in roots.items():
            for n in changed:archive.add(root/n,arcname=label+'/'+n)
    hashes={n:sha(candidate/n) for n in names}
    for root in roots.values():
        for n in names:
            pending=root/(n+'.startup-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n)
        if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('copy_mismatch')
    frozen_unchanged()
    result=dict(status='can_coherent_startup_deployed',tests=count,hashes=hashes,previous_hashes=old,
        model_sha256=model_sha,motion_and_feedback_functions_ast_unchanged=preserved,
        apple_default_behavior_unchanged=True,can_only_opt_in=True,
        source_age_limit_s=.4,startup_stability_window_s=.15,startup_timeout_s=5.,
        motion_command_retries_added=False,navigation_parameters_changed=False,robot_commands_sent=False)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
