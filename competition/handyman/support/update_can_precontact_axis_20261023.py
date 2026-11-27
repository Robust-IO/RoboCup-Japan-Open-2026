"""Deploy tested can-only near tracking; no robot commands."""
import argparse,ast,hashlib,json,os,re,shutil,subprocess,sys,tarfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--sha',required=True);args=p.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
roots={'runtime':Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),'training':repo/'training','module':repo/'training/automation/apple_transfer_v1'}
code=roots['runtime'];prefix='can-precontact-axis-v1-20261023';validation=base/(prefix+'-validation')
names=['held_can_appearance.py','run_canned_grasp_trial.py','canned_grasp_runtime.py','test_tracked_can_cap_geometry.py','test_canned_grasp_geometry.py','OBJECT_TRANSFER.txt']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();read=lambda p:json.loads(p.read_text())
patch=base/(prefix+'-patch.tar.gz')
if sha(patch)!=args.sha:raise ValueError('wrong_patch_hash')
if subprocess.check_output(['git','-C',str(repo),'branch','--show-current'],text=True).strip()!='feat/handyman-phase5-grasping':raise ValueError('wrong_branch')
old={n:sha(code/n) for n in names}
for root in roots.values():
    if any(sha(root/n)!=v for n,v in old.items()):raise ValueError('source_copies_differ')
frozen={label:{x.relative_to(root).as_posix():sha(x) for x in root.rglob('*') if x.is_file()
    and '__pycache__' not in x.parts and not x.name.endswith('.pyc') and x.relative_to(root).as_posix() not in names}
    for label,root in roots.items()}
cfg=read(code/'config/apple_transfer_auto_v1.json');registry=read(Path(cfg['registry']));model_sha=sha(Path(registry['weights']))
validation.mkdir(exist_ok=False);candidate=validation/'candidate'
shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(patch) as archive:
    if set(archive.getnames())!=set(names):raise ValueError('wrong_patch_files')
    for member in archive.getmembers():
        if not member.isfile() or Path(member.name).name!=member.name:raise ValueError('unsafe_archive')
        (candidate/member.name).write_bytes(archive.extractfile(member).read())
for n in names:
    if n.endswith('.py'):ast.parse((candidate/n).read_text())
with (validation/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),stdout=f,stderr=subprocess.STDOUT,timeout=180)
output=(validation/'tests.log').read_text();print(output[-2200:],flush=True)
if proc.returncode:raise ValueError('candidate_tests_failed')
count=int(re.search(r'Ran (\d+) tests',output).group(1));sys.path.insert(0,str(candidate))
import cv2,numpy as np
from canned_grasp_geometry import measure_near,predicted_before_contact
from held_can_appearance import build_appearance
from apple_transfer_automation import RobotLock
trial=base/'can-auto-live-20261023-04/grasp';ref=read(trial/'locked-can-reference.json')
source=Path(read(trial/'result.json')['initial_observations'][-1]['capture'])
def raw(path):return cv2.cvtColor(cv2.imread(str(path/'rgb.png')),cv2.COLOR_BGR2RGB),np.load(path/'depth_m.npy',allow_pickle=False),read(path/'snapshot.json')
rgb,depth,snap=raw(source);appearance=build_appearance(rgb,depth,snap,read(source/'can-identity.json')['geometry'])
rows=[]
for path in sorted((trial/'captures').glob('rejected-geometry-*')):
    rgb,depth,snap=raw(path);expected=predicted_before_contact(ref,snap,ref['task_id'])
    try:measure_near(depth,snap,expected,ref['radius_m'],ref['height_m'])
    except ValueError as exc:
        if str(exc)!='cylinder_radius_outside_observation_envelope':raise
    else:raise ValueError('failed_original_frame_not_reproduced')
    m=measure_near(depth,snap,expected,ref['radius_m'],ref['height_m'],rgb=rgb,appearance=appearance)
    rows.append(dict(capture=str(path),radius_m=m['surface_radius_m'],tilt_deg=m['tilt_deg'],residual_m=m['cylinder_side_residual_p95_m']))
if len(rows)!=5:raise ValueError('five_actual_failure_frames_required')
prior=[]
for run in ('can-auto-live-20261023-01','can-auto-live-20261023-02','can-auto-live-20261023-04'):
    parent=base/run/'grasp'
    for path in sorted((parent/'captures').glob('*')):
        file=path/'can-measurement.json'
        if not file.is_file() or path.name.startswith('after-lift'):continue
        old_m=read(file);rgb,depth,snap=raw(path)
        r=read(parent/'locked-can-reference.json')
        try:current=measure_near(depth,snap,old_m['center_m'],r['radius_m'],r['height_m'])
        except ValueError:continue
        # Numerical legacy algorithm is untouched; known good measurements stay valid.
        if np.linalg.norm(np.asarray(current['center_m'])-old_m['center_m'])>.002:raise ValueError('legacy_replay_changed')
        prior.append(str(path))
if len(prior)<3:raise ValueError('previous_real_near_measurements_required')
def unchanged():
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
    if sha(Path(registry['weights']))!=model_sha:raise ValueError('model_changed')
with RobotLock():
    unchanged()
    for root in roots.values():
        if any(sha(root/n)!=v for n,v in old.items()):raise ValueError('concurrent_source_change')
    with tarfile.open(validation/'previous-files.tar.gz','x:gz') as archive:
        for label,root in roots.items():
            for n in names:archive.add(root/n,arcname=label+'/'+n)
    hashes={n:sha(candidate/n) for n in names}
    for root in roots.values():
        for n in names:
            tmp=root/(n+'.axis-pending')
            with tmp.open('xb') as f:f.write((candidate/n).read_bytes())
            tmp.replace(root/n)
        if any(sha(root/n)!=v for n,v in hashes.items()):raise ValueError('copy_mismatch')
    unchanged()
    result=dict(status='can_precontact_axis_deployed',tests=count,hashes=hashes,previous_hashes=old,
        failed_frames_replayed=rows,previous_good_near_frames=len(prior),model_sha256=model_sha,
        radius_range_relaxed=False,apple_and_navigation_unchanged=True,robot_commands_sent=False)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
