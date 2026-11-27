import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--attempt',type=int,default=1);args=p.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime');name='cigarette-source-20261101-'+format(args.attempt,'02d')
if any((base/(name+suffix)).exists() for suffix in ('','-console.log','-launch.json')):raise ValueError('existing_source_trial_must_be_reviewed')
command=['bash','-lc','source /opt/ros/humble/setup.bash; '
 'source /__HANDYMAN_PACKAGE__/project/install/setup.bash; '
 'export HANDYMAN_RESIDENT_MODEL=1; '
 'export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/'+name+'-roslogs; '
 'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py '
 '--domain 71 --dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml '
 '--run -- /usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/observe_grasp_object_20261101.py '
 '--object cigarette --output '+str(base/name)]
with (base/(name+'-console.log')).open('xb') as log:
    child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(base/name))
(base/(name+'-launch.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
