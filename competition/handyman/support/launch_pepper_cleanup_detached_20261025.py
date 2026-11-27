import subprocess,json,time
from pathlib import Path
base=Path('/__HANDYMAN_PACKAGE__/runtime');log=base/'pepper-third-round-open-cleanup-20261025-01.console.log'
command='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/pepper-third-cleanup-detached-20261025-roslogs /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/finish_pepper_third_round_cleanup_20261025.py'
with log.open('x') as f:
 child=subprocess.Popen(['bash','-lc',command],stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
record=dict(pid=child.pid,launched_ns=time.time_ns(),stdout_file=str(log),independent_of_ssh_stdout=True)
with (base/'pepper-third-round-cleanup-detached-launch-20261025.json').open('x') as f:json.dump(record,f,indent=2)
print(json.dumps(record),flush=True)
