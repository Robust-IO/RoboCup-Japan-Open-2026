"""Reuse the existing pepper adapter and independent new-route configuration."""
import json,subprocess,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');root=b/('pepper-bedroom-kitchen-'+time.strftime('%Y%m%d-%H%M%S'));task=b/'pepper-bedroom-to-kitchen-20261107.json'
cmd='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export HANDYMAN_RESIDENT_MODEL=1; export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; export ROS_LOG_DIR='+str(root)+'-roslogs; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(f/'run_object_transfer.py')+' --object ground_pepper --run --confirm-layout LayoutA --confirm-localization --simulation-ignore-collision --task-config '+str(task)+' --output '+str(root)
with root.with_suffix('.log').open('x') as log:child=subprocess.Popen(['bash','-lc',cmd],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,output=str(root),task=str(task),started_ns=time.time_ns(),object='ground_pepper',source='white_side_table#bedroom_1',destination='dining_table#kitchen')
(b/'pepper-kitchen-live-latest.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
