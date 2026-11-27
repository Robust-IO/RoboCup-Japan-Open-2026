import hashlib,json,os,subprocess,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
name='horizontal-can-carry-20261101-01'
if any((base/(name+suffix)).exists() for suffix in ('','-console.log','-launch.json','-tests.log')):
    raise ValueError('existing_carry_must_be_reviewed')
env=dict(os.environ,PYTHONPATH=str(base)+':'+str(code))
with (base/(name+'-tests.log')).open('xb') as out:
    result=subprocess.run(['/usr/bin/python3','-m','unittest','test_horizontal_can_transport','test_horizontal_can_hand',
        'test_horizontal_can_geometry','test_horizontal_can_grasp_plan'],cwd=base,env=env,stdout=out,stderr=subprocess.STDOUT,timeout=45)
if result.returncode:raise ValueError('carry_geometry_tests_failed')
reference=json.loads((base/'phase5-horizontal-can-held-progress-20261101.json').read_text())
for relative,sha in reference['latest_expected_hashes'].items():
    if hashlib.sha256((code/relative).read_bytes()).hexdigest()!=sha:raise ValueError('frozen_runtime_changed:'+relative)
command=['bash','-lc','source /opt/ros/humble/setup.bash; '
 'source /__HANDYMAN_PACKAGE__/project/install/setup.bash; '
 'export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; '
 'export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/horizontal-can-carry-20261101-01-roslogs; '
 'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py '
 '--domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml '
 '--run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/horizontal_can_carry.py '
 '--run --output /__HANDYMAN_PACKAGE__/runtime/horizontal-can-carry-20261101-01']
with (base/(name+'-console.log')).open('xb') as log:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(base/name),tests_passed=21)
(base/(name+'-launch.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
