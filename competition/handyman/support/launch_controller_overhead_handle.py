import json,subprocess,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');root=b/('controller-overhead-handle-launch-'+time.strftime('%Y%m%d-%H%M%S'))
cmd='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime/generic-grasp-v1:/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; export ROS_LOG_DIR='+str(root)+'-roslogs; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/generic-grasp-v1/run_controller_overhead_handle.py # '+str(root)
with root.with_suffix('.log').open('x') as log:p=subprocess.Popen(['bash','-lc',cmd],stdout=log,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
a=dict(pid=p.pid,output=str(root),target='game_controller');(b/'controller-overhead-handle-launch-latest.json').write_text(json.dumps(a,indent=2));print(json.dumps(a))
