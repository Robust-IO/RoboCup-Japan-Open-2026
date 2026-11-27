import json,subprocess,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');name='horizontal-can-grasp-20261101-02'
if not (base/'horizontal-can-validation-20261101-02.json').exists():raise ValueError('candidate_validation_required')
if any((base/(name+suffix)).exists() for suffix in ('','-launch.json','-console.log')):raise ValueError('existing_trial_must_be_reviewed')
command=['bash','-lc','source /opt/ros/humble/setup.bash; '
    'source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash; '
    'export HANDYMAN_RESIDENT_MODEL=1; export PYTHONPATH=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1:$PYTHONPATH; '
    'export ROS_LOG_DIR=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal-can-grasp-20261101-02-roslogs; '
    'exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py '
    '--domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml '
    '--run -- /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal_can_trial.py '
    '--run --output /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal-can-grasp-20261101-02 '
    '--continue-open-overhead /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal-can-grasp-20261101-01']
with (base/(name+'-console.log')).open('xb') as log:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(base/name),detached=True)
(base/(name+'-launch.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
