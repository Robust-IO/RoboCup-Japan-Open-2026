"""One explicitly reviewed NEW close/lift; never replays the stopped full trial."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,check_inputs
from apple_transfer_auto_plan import map_pose

p=argparse.ArgumentParser();p.add_argument('--run',action='store_true');a=p.parse_args()
if not a.run or os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
    raise ValueError('explicit_domain71_live_run_required')
base=Path('/__HANDYMAN_PACKAGE__/runtime')
out=base/'apple-gate-pruning-live-close-lift-20261018-01'
startup=base/(out.name+'.startup.json');console=base/(out.name+'.console.log')
if any(path.exists() for path in (out,startup,console)):
    raise ValueError('new_output_required_no_replay')
capture=base/'apple-gate-pruning-live-precheck-20261018-01/000005'
s=json.loads((capture/'snapshot.json').read_text());j=s['joints']
if (not .24<=j['arm_lift_joint']<=.30 or abs(j['hand_motor_joint']-.65)>.005
        or any(abs(j[key]-v)>.006 for key,v in dict(arm_flex_joint=-1.7,arm_roll_joint=0.,
            wrist_flex_joint=.1292036732051034,wrist_roll_joint=0.,head_pan_joint=0.,head_tilt_joint=-.5).items())):
    raise ValueError('visually_reviewed_prepared_open_pose_required')
config=json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
pose=map_pose(s);task=out.name
with RobotLock():
    inputs=check_inputs(config)
    command=['/usr/bin/python3',str(code/'run_apple_close_lift_trial.py'),'--run',
        '--model-python',config['model_python'],'--registry',config['registry'],
        '--environment',config['environment'],'--map-pose',*map(str,pose),
        '--room','bedroom','--contact-start-lift',str(j['arm_lift_joint']),
        '--tracking-task-id',task,'--confirm-apple-roi','--verify-preclose-rgbd-follow',
        '--motion-profile',config['motion_profile'],'--output',str(out)]
    with console.open('x') as log:
        child=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
            start_new_session=True,env=dict(os.environ,ROS_LOG_DIR=str(base/(out.name+'-roslogs'))))
    record=dict(pid=child.pid,command=command,output=str(out),console=str(console),
        reviewed_head_and_hand_capture=str(capture),reviewed_map_pose_reference=pose,
        reference_is_not_a_live_identity_lease=True,
        new_owned_observer_and_actual_publication_identity_check_required=True,
        input_files_audit=inputs,base_navigation_or_approach_commands_requested=False,
        old_trial_replayed=False,automatic_retry=False)
    startup.write_text(json.dumps(record,indent=2));print(json.dumps(record),flush=True)
    rc=child.wait()
    result=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {}
    verified=rc==0 and result.get('effective_grasp_success_verified') is True
    print(json.dumps(dict(status='reviewed_new_grasp_verified' if verified else 'stopped_requires_physical_state_audit',
        child_returncode=rc,result=str(out/'result.json'),canonical_status=result.get('status'),
        effective_grasp_success_verified=result.get('effective_grasp_success_verified'),
        whole_uninterrupted_transfer_verified=False)),flush=True)
    if not verified:raise SystemExit(1)
