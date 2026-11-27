import json,subprocess,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');name='horizontal-can-grasp-20261101-02'
if not (base/'horizontal-can-validation-20261101-02.json').exists():raise ValueError('candidate_validation_required')
if any((base/(name+suffix)).exists() for suffix in ('','-launch.json','-console.log')):raise ValueError('existing_trial_must_be_reviewed')
command=['bash','-lc','source /opt/ros/humble/setup.bash; '
    'source /__HANDYMAN_PACKAGE__/project/install/setup.bash; '
    'export HANDYMAN_RESIDENT_MODEL=1; export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; '
    'export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/horizontal-can-grasp-20261101-02-roslogs; '
    'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py '
    '--domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml '
    '--run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/horizontal_can_trial.py '
    '--run --output /__HANDYMAN_PACKAGE__/runtime/horizontal-can-grasp-20261101-02 '
    '--continue-open-overhead /__HANDYMAN_PACKAGE__/runtime/horizontal-can-grasp-20261101-01']
with (base/(name+'-console.log')).open('xb') as log:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(base/name),detached=True)
(base/(name+'-launch.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
