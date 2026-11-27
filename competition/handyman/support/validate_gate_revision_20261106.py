"""Unpack and validate the candidate without ROS initialization or deployment."""
import ast,hashlib,json,os,re,subprocess,tarfile,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');root=b/('gate-revision-v2-'+time.strftime('%Y%m%d-%H%M%S'));root.mkdir()
with tarfile.open(b/'handyman-gate-revision-v2.tar.gz','r:gz') as tar:
    for member in tar.getmembers():
        if not (root/member.name).resolve().is_relative_to(root.resolve()) or member.issym() or member.islnk():raise ValueError('invalid_package_member')
    tar.extractall(root)
manifest=json.loads((root/'package-manifest.json').read_text())
for row in manifest['files']:
    p=root/row['relative']
    if hashlib.sha256(p.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('package_hash_mismatch')
    if p.suffix=='.py':ast.parse(p.read_text())
env=dict(os.environ,PYTHONPATH=':'.join(map(str,[root/'runtime',root/'frozen',b,Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')])));
report=dict(status='validating',root=str(root),robot_commands_sent=False,ros_initialized=False,deployed=False,checks=[])
commands=[('new-behavior-tests',['/usr/bin/python3','-m','unittest','discover','-s',str(root/'tests'),'-p','test_gate_revision.py','-v']),
 ('existing-geometry-tests',['/usr/bin/python3','-m','unittest','test_thin_object_table_safety','test_table_approach_reachability','test_flat_box_place_geometry','test_outcome_transfer_policy']),
 ('recorded-transfer-replay',['/usr/bin/python3',str(root/'tests/replay_recorded_transfer.py'),str(root)])]
for name,command in commands:
    with (root/(name+'.log')).open('x') as stream:result=subprocess.run(command,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=120)
    log=(root/(name+'.log')).read_text(errors='replace');match=re.search(r'Ran (\d+) tests?',log)
    report['checks'].append(dict(name=name,returncode=result.returncode,tests=int(match.group(1)) if match else None,log=str(root/(name+'.log'))))
    if result.returncode:
        report.update(status='validation_failed',failed_check=name,tail=log[-2400:]);break
else:report['status']='offline_validated_not_deployed'
(root/'validation.json').write_text(json.dumps(report,indent=2));(b/'gate-revision-validation-latest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
