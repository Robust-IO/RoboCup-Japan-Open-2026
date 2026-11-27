"""Deploy robot-relative upright-can calibration after tests and real replay."""
import argparse,ast,hashlib,json,math,os,re,shutil,subprocess,sys,tarfile
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--sha',required=True);args=p.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
roots={'runtime':Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),'training':repo/'training','module':repo/'training/automation/apple_transfer_v1'}
code=roots['runtime'];prefix='can-upright-contact-v2-20261024';v=base/(prefix+'-validation')
names=['canned_grasp_geometry.py','canned_grasp_runtime.py','run_canned_grasp_trial.py','test_canned_grasp_geometry.py','OBJECT_TRANSFER.txt']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();read=lambda p:json.loads(p.read_text())
previous=read(base/'canned-upright-reset-required-progress-20261024.json');expected=previous['latest_expected_hashes']
patch=base/(prefix+'-patch.tar.gz')
if sha(patch)!=args.sha:raise ValueError('patch_hash_mismatch')
for root in roots.values():
    if any(sha(root/n)!=digest for n,digest in expected.items()):raise ValueError('tested_source_changed')
old={n:sha(code/n) for n in names};allowed={(root/n).resolve() for root in roots.values() for n in names}
frozen={label:{p.relative_to(root).as_posix():sha(p) for p in root.rglob('*') if p.is_file()
    and '__pycache__' not in p.parts and not p.name.endswith('.pyc') and p.resolve() not in allowed}
    for label,root in roots.items()}
v.mkdir(exist_ok=False);candidate=v/'candidate';shutil.copytree(code,candidate,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
with tarfile.open(patch) as archive:
    if set(archive.getnames())!=set(names):raise ValueError('unexpected_patch_files')
    for m in archive.getmembers():
        if not m.isfile() or Path(m.name).name!=m.name:raise ValueError('unsafe_archive')
        (candidate/m.name).write_bytes(archive.extractfile(m).read())
for n in names:
    if n.endswith('.py'):ast.parse((candidate/n).read_text())
with (v/'tests.log').open('x') as f:
    proc=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(candidate),'-p','test_*.py'],cwd=candidate,
        env=dict(os.environ,PYTHONPATH=str(candidate)+':'+str(candidate/'scripts')),stdout=f,stderr=subprocess.STDOUT,timeout=180)
out=(v/'tests.log').read_text();print(out[-2400:],flush=True)
if proc.returncode:raise ValueError('candidate_tests_failed')
count=int(re.search(r'Ran (\d+) tests',out).group(1));sys.path.insert(0,str(candidate))
import cv2,numpy as np
from canned_grasp_geometry import CONTACT_OFFSET_BASE,measure_near,upright_trial_evidence
from held_can_appearance import build_appearance
from apple_transfer_automation import RobotLock
source=base/'can-auto-live-20261023-05/grasp/captures/preclose'
snap=read(source/'snapshot.json');prior=read(source/'can-measurement.json')
rgb=cv2.cvtColor(cv2.imread(str(source/'rgb.png')),cv2.COLOR_BGR2RGB);depth=np.load(source/'depth_m.npy')
a=build_appearance(rgb,depth,snap,prior)
m=measure_near(depth,snap,prior['center_m'],prior['footprint_radius_m'],prior['height_m'],rgb=rgb,appearance=a)
upright=upright_trial_evidence(m)
offset=np.asarray(m['center_m'])-snap['palm_base_at_depth']
if np.linalg.norm(offset[:2]-CONTACT_OFFSET_BASE[:2])>.0005:raise ValueError('calibration_does_not_match_actual_upright_success')
after=read(base/'can-auto-live-20261023-05/held-context.json');axis=np.asarray(after['can_axis_base_m'])
with RobotLock():
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
        if any(sha(root/n)!=digest for n,digest in old.items()):raise ValueError('concurrent_edit')
    with tarfile.open(v/'previous-files.tar.gz','x:gz') as archive:
        for label,root in roots.items():
            for n in names:archive.add(root/n,arcname=label+'/'+n)
    hashes={n:sha(candidate/n) for n in names}
    for root in roots.values():
        for n in names:
            pending=root/(n+'.upright-pending')
            with pending.open('xb') as f:f.write((candidate/n).read_bytes())
            pending.replace(root/n)
        if any(sha(root/n)!=digest for n,digest in hashes.items()):raise ValueError('copy_mismatch')
    for label,root in roots.items():
        if any(sha(root/n)!=digest for n,digest in frozen[label].items()):raise ValueError('unrelated_source_changed')
    result=dict(status='can_upright_contact_calibration_deployed',tests=count,hashes=hashes,previous_hashes=old,
        actual_success_source=str(source),source_raw_sha256={n:sha(source/n) for n in ('rgb.png','depth_m.npy','snapshot.json')},
        measured_source_preclose_tilt_deg=m['tilt_deg'],calibrated_relative_offset_base_m=CONTACT_OFFSET_BASE.tolist(),
        recorded_success_afterlift_tilt_deg=float(math.degrees(math.acos(np.clip(axis[2],-1,1)))),
        world_coordinate_replay=False,original_navigation_model_and_apple_unchanged=True,robot_commands_sent=False,
        live_upright_regression_still_required=True)
    with (base/(prefix+'-deployment.json')).open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
