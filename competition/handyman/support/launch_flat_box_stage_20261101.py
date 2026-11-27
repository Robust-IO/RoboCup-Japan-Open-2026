import argparse,json,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('stage',choices=['contact','carry','place','review','reset','cleanup','reposition','readiness','bedroom-entry','room-scan']);p.add_argument('--attempt',type=int,default=1);p.add_argument('--grasp');p.add_argument('--held-run');p.add_argument('--gripper-geometry');p.add_argument('--previous');p.add_argument('--destination-id');args=p.parse_args()
base=Path('/__HANDYMAN_PACKAGE__/runtime');name='cigarette-'+args.stage+'-20261101-'+format(args.attempt,'02d')
if any((base/(name+s)).exists() for s in ('','-console.log','-launch.json')):raise ValueError('existing_trial_must_be_reviewed')
command=['bash','-lc','source /opt/ros/humble/setup.bash; source /__HANDYMAN_PACKAGE__/project/install/setup.bash; '
 'export HANDYMAN_RESIDENT_MODEL=1; export PYTHONPATH=/__HANDYMAN_PACKAGE__/runtime:/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1:$PYTHONPATH; '
 'export ROS_LOG_DIR=/__HANDYMAN_PACKAGE__/runtime/'+name+'-roslogs; '
 'exec /usr/bin/python3 /__HANDYMAN_PACKAGE__/project/training/scripts/cyclone_test_env.py --domain 71 '
 '--dds-config /__HANDYMAN_PACKAGE__/project/training/config/cyclone_search_stack.xml --run -- '
 '/usr/bin/python3 /__HANDYMAN_PACKAGE__/runtime/'+({'review':'review_flat_box_place_20261101.py','reset':'reset_flat_box_for_retest_20261101.py','cleanup':'finish_flat_box_reset_20261101.py','readiness':'inspect_cigarette_room_task_20261101.py','bedroom-entry':'navigate_cigarette_bedroom_20261101.py','room-scan':'cigarette_room_scan.py'}.get(args.stage,'flat_box_'+args.stage+'.py'))+' --run --output '+str(base/name)+(' --grasp '+args.grasp if args.grasp else '')+(' --held-run '+args.held_run if args.held_run else '')+(' --gripper-geometry '+args.gripper_geometry if args.gripper_geometry else '')+(' --previous '+args.previous if args.previous else '')+(' --destination-id '+args.destination_id if args.destination_id else '')]
with (base/(name+'-console.log')).open('xb') as log:child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(base/name));(base/(name+'-launch.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
