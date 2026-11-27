import hashlib,json,subprocess,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');name='horizontal-can-place-20261101-01'
if any((base/(name+suffix)).exists() for suffix in ('','-console.log','-launch.json')):
    raise ValueError('existing_placement_must_be_reviewed')
carry=json.loads((base/'horizontal-can-carry-20261101-01/result.json').read_text())
if carry.get('carry_destination_verified') is not True or carry.get('release_commands_sent')!=0:
    raise ValueError('verified_destination_held_can_required')
subprocess.run(['/usr/bin/python3','-m','py_compile',str(base/'horizontal_can_place.py')],check=True)
command=['bash','-lc','source /opt/ros/humble/setup.bash; '
 'source /__HANDYMAN_PACKAGE__/project/install/setup.bash; '
 'export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; '
 'export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/horizontal-can-place-20261101-01-roslogs; '
 'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py '
 '--domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml '
 '--run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/horizontal_can_place.py '
 '--run --output /__HANDYMAN_PACKAGE__/runtime/horizontal-can-place-20261101-01']
with (base/(name+'-console.log')).open('xb') as log:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(base/name),
    candidate_sha256=hashlib.sha256((base/'horizontal_can_place.py').read_bytes()).hexdigest())
(base/(name+'-launch.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
