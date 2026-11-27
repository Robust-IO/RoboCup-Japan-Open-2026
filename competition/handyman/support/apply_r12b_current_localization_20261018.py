"""Replace only the EXACT inspected idle anchor; no navigation/robot commands."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

code=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1');sys.path.insert(0,str(code))
from apple_transfer_automation import RobotLock,write_json
base=Path('/__HANDYMAN_PACKAGE__/runtime');out=base/'apple-r12b-current-localization-20261018-01'
match=base/'apple-r12b-current-localization-audit-20261018.json'
if os.environ.get('ROS_DOMAIN_ID')!='71' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise ValueError('domain71_required')
with RobotLock():
    out.mkdir(exist_ok=False)
    subprocess.run([sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(out/'before-ready.json')],check=True)
    parent=Path('/proc/3630/cmdline').read_bytes().replace(b'\0',b' ').decode()
    child=Path('/proc/3631/cmdline').read_bytes().replace(b'\0',b' ').decode()
    if ('ros2 run handyman_rebuild_ros2 handyman_initial_pose_localizer' not in parent
        or '/install/handyman_rebuild_ros2/lib/handyman_rebuild_ros2/handyman_initial_pose_localizer' not in child
        or os.getpgid(3630)!=3630 or os.getpgid(3631)!=3630):raise ValueError('exact_current_anchor_identity_changed')
    command=[sys.executable,str(base/'publish_r12b_reviewed_map_anchor_20261018.py'),'--run',
             '--match',str(match),'--output',str(out/'corrected-map-anchor.json')]
    intent=dict(status='replacing_exact_idle_anchor',old_parent_pid=3630,old_child_pid=3631,
        old_parent_command=parent,old_child_command=child,new_command=command,robot_commands_sent=False,
        navigation_parameters_changed=False,old_numeric_anchor_restored=False)
    write_json(out/'startup.json',intent)
    os.killpg(3630,signal.SIGINT)
    end=time.monotonic()+5
    while (Path('/proc/3630').exists() or Path('/proc/3631').exists()) and time.monotonic()<end:time.sleep(.05)
    if Path('/proc/3630').exists() or Path('/proc/3631').exists():raise ValueError('old_anchor_not_stopped_no_duplicate')
    with (out/'corrected-map-anchor.log').open('x') as log:
        publisher=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    intent['new_anchor_pid']=publisher.pid;write_json(out/'startup.json',intent)
    end=time.monotonic()+12
    while time.monotonic()<end:
        if publisher.poll() is not None:raise ValueError('new_anchor_exited_no_motion')
        if (out/'corrected-map-anchor.json').exists():break
        time.sleep(.1)
    else:raise ValueError('new_anchor_publication_not_confirmed')
    subprocess.run([sys.executable,str(base/'source-table-boundary-audit-20261017-01/capture_table_localization_audit.py'),
        '--output',str(out/'new-scans.json')],check=True)
    subprocess.run([sys.executable,str(code/'inspect_transfer_ready.py'),'--output',str(out/'ready.json')],check=True)
    intent.update(status='current_idle_localization_correction_published_and_readiness_verified',
        alignment=str(out/'corrected-map-anchor.json'),new_scans=str(out/'new-scans.json'),
        requires_new_scan_map_check_before_full_trial=True,robot_commands_sent=False)
    write_json(out/'completion.json',intent);print(json.dumps(intent),flush=True)
