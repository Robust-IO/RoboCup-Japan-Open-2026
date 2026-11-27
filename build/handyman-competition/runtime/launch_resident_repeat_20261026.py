import json,subprocess,time
from pathlib import Path
b=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');name='penguin-resident-repeat-20261026-01'
receipt=b/(name+'-launch.json');log=b/(name+'-console.log')
if any(p.exists() for p in (receipt,log,b/name)):raise RuntimeError('existing_repeat_must_be_reviewed')
script='source /opt/ros/humble/setup.bash; source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash; export HANDYMAN_RESIDENT_MODEL=1; export ROS_LOG_DIR=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/resident-repeat-roslogs; exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/repeat_resident_timing_20261026.py'
with log.open('xb') as out:
    child=subprocess.Popen(['bash','-lc',script],stdin=subprocess.DEVNULL,stdout=out,stderr=subprocess.STDOUT,start_new_session=True,cwd='/home/crazylearner')
row=dict(pid=child.pid,started_ns=time.time_ns(),output=str(b/name),restore_is_not_in_official_time=True,detached=True)
receipt.write_text(json.dumps(row,indent=2));print(json.dumps(row))
