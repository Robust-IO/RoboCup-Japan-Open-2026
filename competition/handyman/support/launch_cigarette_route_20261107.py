"""Launch the explicitly confirmed new route; --run and --source-ready required."""
import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');p.add_argument('--source-ready',action='store_true');a=p.parse_args()
b=Path('/__HANDYMAN_PACKAGE__/runtime');task=b/'task-bedroom-to-kitchen.json'
if not a.run or not a.source_ready:raise ValueError('explicit_run_and_source_ready_required')
root=b/('cigarette-new-route-'+time.strftime('%Y%m%d-%H%M%S'))
cmd='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export HANDYMAN_RESIDENT_MODEL=1; export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; export ROS_LOG_DIR='+str(root)+'-roslogs; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(b/'run_cigarette_route_20261107.py')+' --run --source-ready --task '+str(task)+' --output '+str(root)
with root.with_suffix('.log').open('x') as log:child=subprocess.Popen(['bash','-lc',cmd],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,output=str(root),task=str(task),started_ns=time.time_ns(),source_ready_confirmed=True)
(b/'cigarette-new-route-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
