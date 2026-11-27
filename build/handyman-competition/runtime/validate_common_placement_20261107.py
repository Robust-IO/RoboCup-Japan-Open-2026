"""Offline tests and recorded kitchen replay. No ROS node or robot command."""
import ast,hashlib,json,os,re,subprocess,sys,tarfile,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');f=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
root=b/('common-placement-v1-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir()
with tarfile.open(b/'handyman-common-placement-v1.tar.gz') as t:
    for m in t.getmembers():
        if not (root/m.name).resolve().is_relative_to(root.resolve()) or m.issym() or m.islnk():raise ValueError('unsafe_member')
    t.extractall(root)
manifest=json.loads((root/'package-manifest.json').read_text())
for row in manifest['files']:
    p=root/row['relative']
    if hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('package_hash_mismatch')
    if p.suffix=='.py':ast.parse(p.read_text())
paths=[str(root/'runtime'),str(root/'frozen'),str(b),str(f)]
env=dict(os.environ,PYTHONPATH=':'.join(paths))
groups=[('common-behavior',['discover','-s',str(root/'tests'),'-v']),
    ('existing-regressions',['test_inverse_table_place_plan','test_motion_speed','test_pepper_tasks','test_pepper_geometry','test_pepper_transfer','test_thin_object_table_safety','test_table_approach_reachability','test_flat_box_place_geometry','test_outcome_transfer_policy','test_tabletop_support','-v'])]
report=dict(status='validating',root=str(root),checks=[],tests=0,robot_commands_sent=False,ros_initialized=False)
for name,args in groups:
    log=root/(name+'.log')
    with log.open('x') as stream:r=subprocess.run([sys.executable,'-m','unittest',*args],env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=150)
    out=log.read_text();m=re.search(r'Ran (\d+) tests?',out);n=int(m[1]) if m else 0
    report['checks'].append(dict(name=name,returncode=r.returncode,tests=n,log=str(log)));report['tests']+=n
    if r.returncode:
        report.update(status='failed',tail=out[-5000:]);break
else:
    sys.path[:0]=paths
    import numpy as np
    from can_destination_surface import measure
    from pepper_place import make_plan,extension_clearance
    from apple_transfer_auto_plan import map_point,map_pose
    from pregrasp_geometry import expected_horizontal_center
    from table_approach_reachability import base_path
    from hand_camera_ray import rotation
    from table_transfer_core import lift_targets,release_height,extension_retreat
    original=b/'pepper-bedroom-kitchen-20261107-153843'
    capture=original/'place/captures/table-before-extension'
    s=json.loads((capture/'snapshot.json').read_text());depth=np.load(capture/'depth_m.npy')
    held=json.loads((original/'held-context.json').read_text());cfg=json.loads((original/'config.snapshot.json').read_text())
    dest=next(d for d in json.loads(Path(cfg['semantic_map']).read_text())['destinations'] if d['id']==cfg['destination_id'])
    plane=measure(depth,s,dest);plan=make_plan(depth,s,dest,plane,held,cfg,'common-recorded-kitchen-replay')
    pred=map_point(s,expected_horizontal_center(plan['palm_height_m'],held['body_minus_palm_horizontal_m']))
    clearance=extension_clearance(pred,s,plane,plan)
    delta=rotation(s['tracking_transforms']['map_from_base_at_depth']['quaternion']).T@np.r_[np.array(plan['release_xy_map_m'])-pred[:2],0.]
    reach=base_path(map_pose(s),[*delta[:2],0.],dest)
    heights=lift_targets(plan['palm_height_m'])
    if not heights['release_lift_m']>.443:raise ValueError('recorded_high_table_not_replayed')
    # Held offset stays fixed while synthetic support heights change; recorded
    # depth above is the real kitchen proof. These are geometry-only scenarios.
    bottom=held['last_horizontal_measurement']['bottom_vertical_extent_m']-held['body_minus_palm_horizontal_m'][2]
    scenarios=[dict(table_height_m=h,**lift_targets(release_height(h,bottom,cfg['policy']['release_gap_m']))) for h in (.4,.6,.75)]
    replay=dict(status='recorded_high_table_replayed',capture=str(capture),plan=plan,dynamic_heights=heights,
        extension_clearance=clearance,measured_retreat_m=extension_retreat(clearance),base_reachability=reach,
        synthetic_height_scenarios=scenarios,robot_commands_sent=False)
    (root/'recorded-kitchen-replay.json').write_text(json.dumps(replay,indent=2))
    report.update(status='offline_validated_not_deployed',recorded_replay=str(root/'recorded-kitchen-replay.json'))
(root/'validation.json').write_text(json.dumps(report,indent=2))
(b/'common-placement-validation-latest.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
