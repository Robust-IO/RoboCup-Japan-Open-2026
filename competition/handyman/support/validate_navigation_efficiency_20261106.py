"""Validate a staged package, without ROS/control initialization."""
import ast,hashlib,json,os,re,subprocess,tarfile,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');root=b/('navigation-efficiency-v1-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir()
with tarfile.open(b/'handyman-navigation-efficiency-v1.tar.gz') as t:
    for m in t.getmembers():
        if not (root/m.name).resolve().is_relative_to(root.resolve()) or m.issym() or m.islnk():raise ValueError('unsafe_package_member')
    t.extractall(root)
manifest=json.loads((root/'package-manifest.json').read_text())
for row in manifest['files']:
    p=root/row['relative']
    if hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('package_hash_mismatch')
    if p.suffix=='.py':ast.parse(p.read_text())
env=dict(os.environ,PYTHONPATH=':'.join(map(str,[root/'frozen',root/'runtime',b,Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')])))
commands=[('efficiency-and-v2-regressions',['/usr/bin/python3','-m','unittest','discover','-s',str(root/'tests'),'-v']),
    ('speed-feedback-regressions',['/usr/bin/python3','-m','unittest','test_motion_speed','test_navigation_feedback_gate','test_navigation_feedback_recovery','-v']),
    ('grasp-place-regressions',['/usr/bin/python3','-m','unittest','test_thin_object_table_safety','test_table_approach_reachability','test_flat_box_place_geometry','test_outcome_transfer_policy','-v'])]
report=dict(status='validating',root=str(root),tests=0,checks=[],robot_commands_sent=False,ros_initialized=False)
for name,cmd in commands:
    log=root/(name+'.log')
    with log.open('x') as stream:r=subprocess.run(cmd,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=120)
    text=log.read_text();match=re.search(r'Ran (\d+) tests?',text);count=int(match.group(1)) if match else 0
    report['checks'].append(dict(name=name,returncode=r.returncode,tests=count,log=str(log)));report['tests']+=count
    if r.returncode:report.update(status='failed',tail=text[-3500:]);break
else:report['status']='offline_validated_not_deployed'
(root/'validation.json').write_text(json.dumps(report,indent=2));(b/'navigation-efficiency-validation-latest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
