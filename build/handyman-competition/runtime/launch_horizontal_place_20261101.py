import hashlib,json,subprocess,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');name='horizontal-can-place-20261101-01'
if any((base/(name+suffix)).exists() for suffix in ('','-console.log','-launch.json')):
    raise ValueError('existing_placement_must_be_reviewed')
carry=json.loads((base/'horizontal-can-carry-20261101-01/result.json').read_text())
if carry.get('carry_destination_verified') is not True or carry.get('release_commands_sent')!=0:
    raise ValueError('verified_destination_held_can_required')
subprocess.run(['/usr/bin/python3','-m','py_compile',str(base/'horizontal_can_place.py')],check=True)
command=['bash','-lc','source /opt/ros/humble/setup.bash; '
 'source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash; '
 'export PYTHONPATH=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1:$PYTHONPATH; '
 'export ROS_LOG_DIR=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal-can-place-20261101-01-roslogs; '
 'exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py '
 '--domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml '
 '--run -- /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal_can_place.py '
 '--run --output /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal-can-place-20261101-01']
with (base/(name+'-console.log')).open('xb') as log:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(base/name),
    candidate_sha256=hashlib.sha256((base/'horizontal_can_place.py').read_bytes()).hexdigest())
(base/(name+'-launch.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
