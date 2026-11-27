"""User-authorized NEW full timed trial. No old action replay or automatic retry."""
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

BASE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime');CODE=Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
CONTROL=BASE/'apple-r17-full-live-control-20261022-01'
OUTPUT=BASE/'automatic-apple-r17-20261022-01'
TASK='apple-r17-full-live-20261022-01'
if CONTROL.exists() or OUTPUT.exists():raise ValueError('new_trial_required_no_replay')
if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('domain71_localhost_required')
deployment=json.loads((BASE/'apple-efficiency-r17-20261022-deployment.json').read_text())
baseline=json.loads((BASE/'apple-efficiency-r17-20261022-baseline.json').read_text())
progress=json.loads((BASE/'apple-efficiency-r17-progress-20261022.json').read_text())
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
if any(sha(CODE/name)!=digest for name,digest in deployment['hashes'].items()):raise ValueError('deployed_code_changed')
if any(sha(CODE/name)!=digest for name,digest in baseline['frozen']['runtime'].items()):raise ValueError('frozen_navigation_or_outcome_code_changed')
if Path('/proc/sys/kernel/random/boot_id').read_text().strip()!=progress['wsl_boot_id']:raise ValueError('new_boot_requires_fresh_initialization')
CONTROL.mkdir()
preparation_started_ns=time.time_ns()
def readonly(name,args,result_path):
    with (CONTROL/(name+'.log')).open('x') as log:
        result=subprocess.run([sys.executable,*map(str,args)],stdout=log,stderr=subprocess.STDOUT,timeout=25)
    if result.returncode:raise RuntimeError('preparation_'+name+'_failed:'+str(result_path))
    return json.loads(result_path.read_text())
ready=readonly('readiness',[CODE/'inspect_transfer_ready.py','--efficient-transfer','--output',CONTROL/'ready.json'],CONTROL/'ready.json')
if not ready['ready']:raise ValueError('fresh_empty_tucked_start_required')
camera=CONTROL/'cameras'
readonly('cameras',[CODE/'capture_grasp_scene.py',camera,'--hand-geometry','--map-geometry',
    '--tracking-epoch',TASK+'-preparation','--duration','7','--interval','.6','--stop-after-frames','1'],camera/'manifest.json')
old=progress['current_robot']['base_map_pose'];current=ready['base_map_pose']
if math.dist(current[:2],old[:2])>.10 or abs(math.atan2(math.sin(current[2]-old[2]),math.cos(current[2]-old[2])))>.10:
    raise ValueError('scene_or_pose_changed_requires_fresh_localization_audit_no_motion_started')
command=[sys.executable,str(CODE/'apple_transfer_automation.py'),'--run',
    '--confirm-layout','LayoutA','--confirm-localization','--simulation-ignore-collision',
    '--task-id',TASK,'--output',str(OUTPUT)]
execution_started_ns=time.time_ns()
with (CONTROL/'automation.log').open('x') as log:
    child=subprocess.Popen(command,env=dict(os.environ,ROS_LOG_DIR=str(CONTROL/'ros-logs')),
        stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
result=dict(status='new_full_timed_transfer_started',pid=child.pid,task_id=TASK,output=str(OUTPUT),
    revision='efficiency-r17',command=command,preparation_started_ns=preparation_started_ns,
    execution_started_ns=execution_started_ns,preparation_elapsed_s=(execution_started_ns-preparation_started_ns)/1e9,
    navigation_unchanged=True,old_actions_replayed=False,automatic_retry=False,
    user_confirmed_source_apple_restored=True,robot_start_pose=current,
    log=str(CONTROL/'automation.log'),timer_basis='Full automation journal begin through verified_complete; internal checks/waits included')
with (CONTROL/'startup.json').open('x') as stream:json.dump(result,stream,indent=2)
print(json.dumps(result),flush=True)
