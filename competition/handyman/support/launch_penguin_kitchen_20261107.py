"""Run the user-requested bedroom penguin -> kitchen dining table task."""
import json,subprocess,time
from pathlib import Path
b=Path('/__HANDYMAN_PACKAGE__/runtime');f=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
task=json.loads((f/'config/penguin_bedroom_to_living_room.json').read_text())
task['name']='toy_penguin_bedroom_to_kitchen_20261107'
task['destination']=dict(room='kitchen',table_id='dining_table#kitchen',
    observation_pose=[9.562303161621093,2.9079999923706055,3.141592653589793])
from_path=b/'penguin-bedroom-to-kitchen-20261107.json'
# Validate before starting any live entry; object coordinates are not supplied.
import sys
sys.path[:0]=[str(b),str(f)]
from penguin_tasks import validate_task
validate_task(task)
from_path.write_text(json.dumps(task,ensure_ascii=False,indent=2))
root=b/('penguin-bedroom-kitchen-'+time.strftime('%Y%m%d-%H%M%S'))
cmd='source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; export HANDYMAN_RESIDENT_MODEL=1; export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; export ROS_LOG_DIR='+str(root)+'-roslogs; exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- /usr/bin/python3 '+str(f/'run_object_transfer.py')+' --object toy_penguin --run --confirm-layout LayoutA --confirm-localization --simulation-ignore-collision --task-config '+str(from_path)+' --output '+str(root)
with root.with_suffix('.log').open('x') as log:
    child=subprocess.Popen(['bash','-lc',cmd],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,output=str(root),task=str(from_path),started_ns=time.time_ns(),object='toy_penguin',
    source='white_side_table#bedroom_1',destination='dining_table#kitchen',uses_common_dynamic_table_height=True)
(b/'penguin-kitchen-live-latest.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt))
