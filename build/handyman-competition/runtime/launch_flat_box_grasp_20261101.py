import argparse,json,subprocess,time,os
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--attempt',type=int,default=1);p.add_argument('--source-table-id');p.add_argument('--operator-confirmed-reference');p.add_argument('--gripper-geometry');p.add_argument('--continue-open');p.add_argument('--tracked-source');args=p.parse_args()
base=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');name='cigarette-grasp-20261101-'+format(args.attempt,'02d')
if any((base/(name+s)).exists() for s in ('','-console.log','-launch.json')):raise ValueError('existing_trial_must_be_reviewed')
env=dict(os.environ,PYTHONPATH=str(base)+':/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
subprocess.run(['/usr/bin/python3','-m','unittest','test_flat_box_geometry','-v'],cwd=base,env=env,check=True)
command=['bash','-lc','source /opt/ros/humble/setup.bash; '
 'source /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/install/setup.bash; '
 'export HANDYMAN_RESIDENT_MODEL=1; '
 'export PYTHONPATH=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime:/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1:$PYTHONPATH; '
 'export ROS_LOG_DIR=/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/'+name+'-roslogs; '
 'exec /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/scripts/cyclone_test_env.py '
 '--domain 71 --dds-config /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/project/training/config/cyclone_search_stack.xml '
 '--run -- /usr/bin/python3 /home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/flat_box_grasp.py --run --output '+str(base/name)+(' --source-table-id '+args.source_table_id if args.source_table_id else '')+(' --operator-confirmed-reference '+args.operator_confirmed_reference if args.operator_confirmed_reference else '')+(' --gripper-geometry '+args.gripper_geometry if args.gripper_geometry else '')+(' --continue-open '+args.continue_open if args.continue_open else '')+(' --tracked-source '+args.tracked_source if args.tracked_source else '')]
with (base/(name+'-console.log')).open('xb') as log:
 child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
receipt=dict(pid=child.pid,started_ns=time.time_ns(),command=command,output=str(base/name))
(base/(name+'-launch.json')).write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
