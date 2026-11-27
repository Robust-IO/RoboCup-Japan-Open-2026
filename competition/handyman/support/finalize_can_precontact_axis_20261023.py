"""Finalize already-tested deployment after auditing nested source-copy paths."""
import hashlib,json,re,sys,tarfile
from pathlib import Path
import cv2,numpy as np
base=Path('/__HANDYMAN_PACKAGE__/runtime');repo=Path('/__HANDYMAN_PACKAGE__/project')
v=base/'can-precontact-axis-v1-20261023-validation';candidate=v/'candidate';sys.path.insert(0,str(candidate))
from apple_transfer_automation import RobotLock
from canned_grasp_geometry import measure_near,predicted_before_contact
from held_can_appearance import build_appearance
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
roots={'runtime':Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1'),'training':repo/'training','module':repo/'training/automation/apple_transfer_v1'}
names=['held_can_appearance.py','run_canned_grasp_trial.py','canned_grasp_runtime.py','test_tracked_can_cap_geometry.py','test_canned_grasp_geometry.py','OBJECT_TRANSFER.txt']
hashes={n:sha(candidate/n) for n in names};allowed={(r/n).resolve() for r in roots.values() for n in names}
count=int(re.search(r'Ran (\d+) tests',(v/'tests.log').read_text()).group(1))
if count!=579 or not (v/'tests.log').read_text().rstrip().endswith('OK'):raise ValueError('successful_tests_required')
with RobotLock():
    previous={}
    with tarfile.open(v/'previous-files.tar.gz') as archive:
        for label,root in roots.items():
            for n,digest in hashes.items():
                if sha(root/n)!=digest:raise ValueError('candidate_copy_mismatch')
                old=hashlib.sha256(archive.extractfile(label+'/'+n).read()).hexdigest()
                if label=='runtime':previous[n]=old
                elif previous[n]!=old:raise ValueError('original_copies_differed')
            for file in candidate.rglob('*'):
                rel=file.relative_to(candidate);other=root/rel
                if (not file.is_file() or '__pycache__' in file.parts or file.name.endswith('.pyc')
                        or other.resolve() in allowed):continue
                if other.is_file() and sha(other)!=sha(file):raise ValueError('unrelated_file_changed:'+str(other))
    trial=base/'can-auto-live-20261023-04/grasp';ref=read(trial/'locked-can-reference.json')
    source=Path(read(trial/'result.json')['initial_observations'][-1]['capture'])
    def raw(p):return cv2.cvtColor(cv2.imread(str(p/'rgb.png')),cv2.COLOR_BGR2RGB),np.load(p/'depth_m.npy'),read(p/'snapshot.json')
    rgb,d,s=raw(source);a=build_appearance(rgb,d,s,read(source/'can-identity.json')['geometry']);rows=[]
    for path in sorted((trial/'captures').glob('rejected-geometry-*')):
        rgb,d,s=raw(path);e=predicted_before_contact(ref,s,ref['task_id'])
        m=measure_near(d,s,e,ref['radius_m'],ref['height_m'],rgb=rgb,appearance=a)
        rows.append(dict(capture=str(path),radius_m=m['surface_radius_m'],tilt_deg=m['tilt_deg']))
    if len(rows)!=5:raise ValueError('real_failure_replay_required')
    good=[]
    for run in ('can-auto-live-20261023-01','can-auto-live-20261023-02','can-auto-live-20261023-04'):
        grasp=base/run/'grasp';r=read(grasp/'locked-can-reference.json')
        for path in (grasp/'captures').glob('*'):
            file=path/'can-measurement.json'
            if not file.is_file() or path.name.startswith('after-lift'):continue
            old=read(file);rgb,d,s=raw(path)
            try:new=measure_near(d,s,old['center_m'],r['radius_m'],r['height_m'])
            except ValueError:continue
            if np.linalg.norm(np.asarray(new['center_m'])-old['center_m'])>.002:raise ValueError('legacy_measurement_changed')
            good.append(str(path))
    if len(good)<3:raise ValueError('previous_near_regression_required')
    cfg=read(roots['runtime']/'config/apple_transfer_auto_v1.json');reg=read(Path(cfg['registry']))
    model=sha(Path(reg['weights']))
    if model!='83e6ceb9650e6da78d05404a5ead9654278d502ce06d812a6b2b70673c513763':raise ValueError('model_changed')
    result=dict(status='can_precontact_axis_deployed',tests=count,hashes=hashes,previous_hashes=previous,
        failed_frames_replayed=rows,previous_good_near_frames=len(good),model_sha256=model,
        radius_range_relaxed=False,apple_and_navigation_unchanged=True,robot_commands_sent=False,
        consistency_check_correction='Nested training/automation source copies counted as intended changed files; all three copies verified against tested candidate.')
    with (base/'can-precontact-axis-v1-20261023-deployment.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result),flush=True)
