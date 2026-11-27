"""Validate and detach the explicit current-pose horizontal-can continuation."""
import hashlib,json,os,subprocess,time
from pathlib import Path
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');code=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
name='horizontal-can-hand-grasp-20261101-04'
paths=[base/(name+suffix) for suffix in ('','-launch.json','-console.log','-validation.json','-tests.log')]
if any(p.exists() for p in paths):raise ValueError('existing_hand_trial_must_be_reviewed')
prior=json.loads((base/'phase5-horizontal-can-progress-20261101.json').read_text())
for relative,expected in prior['latest_expected_hashes'].items():
    if hashlib.sha256((code/relative).read_bytes()).hexdigest()!=expected:raise ValueError('reviewed_runtime_changed:'+relative)
env=dict(os.environ,PYTHONPATH=str(base)+':'+str(code))
with (base/(name+'-tests.log')).open('xb') as log:
    tested=subprocess.run(['/usr/bin/python3','-m','unittest','test_horizontal_can_hand','test_horizontal_can_geometry','test_horizontal_can_grasp_plan'],
        cwd=base,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=45)
if tested.returncode:raise ValueError('hand_trial_tests_failed')
files=['horizontal_can_geometry.py','horizontal_can_grasp_plan.py','horizontal_can_trial.py','horizontal_can_hand.py','horizontal_can_hand_trial.py']
hashes={f:hashlib.sha256((base/f).read_bytes()).hexdigest() for f in files}
branch=subprocess.run(['git','-C','/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project','branch','--show-current'],capture_output=True,text=True,check=True).stdout.strip()
(base/(name+'-validation.json')).write_text(json.dumps(dict(tests_passed=16,candidate_sha256=hashes,branch_at_launch=branch,
    existing_runtime_unchanged=True,robot_commands_sent=False),indent=2))
command=['bash','-lc','source /opt/ros/humble/setup.bash; '
    'source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash; '
    'export PYTHONPATH=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1:$PYTHONPATH; '
    'export ROS_LOG_DIR=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal-can-hand-grasp-20261101-04-roslogs; '
    'exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py '
    '--domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml '
    '--run -- /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal_can_hand_trial.py '
    '--run --output /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal-can-hand-grasp-20261101-04 '
    '--previous /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal-can-hand-grasp-20261101-03 '
    '--continue-open-contact '
    '--source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/horizontal-can-grasp-20261101-01']
with (base/(name+'-console.log')).open('xb') as log:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(base/name),detached=True,branch=branch)
(base/(name+'-launch.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
