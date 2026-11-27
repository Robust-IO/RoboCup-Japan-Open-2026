"""Launch the isolated new entry without altering the user's default ROS env."""
import os
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
B=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime')
F=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
env=dict(os.environ)
env['AUTOGRASP_ROOT']=str(HERE)
env['AUTOGRASP_RUNTIME_PATHS']=os.pathsep.join(map(str,[HERE,HERE.parent,B/'generic-grasp-v1',B,F]))
env['ROS_LOG_DIR']=str(HERE/'roslogs')
command='''source /opt/ros/humble/setup.bash
source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash
export PYTHONPATH="$AUTOGRASP_RUNTIME_PATHS:$PYTHONPATH"
exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 "$AUTOGRASP_ROOT/run_autograsp.py" "$@"
'''
args=['--config',str(F/'config/apple_transfer_auto_v1.json'),
      '--hand',str(B/'autograsp-v1/replay-inputs/hand.json'),*sys.argv[1:]]
subprocess.run(['/bin/bash','-c',command,'autograsp',*args],env=env,check=True)
