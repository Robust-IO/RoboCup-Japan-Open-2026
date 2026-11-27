"""Establish this restart's initial anchor and collect read-only readiness evidence."""
import json
import os
from pathlib import Path
import subprocess
import time
import yaml
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

if os.environ.get('ROS_DOMAIN_ID') != '71':
    raise ValueError('domain71_required')
root = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/apple-initial-readiness-20261018-01')
code = Path('/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/tools/apple-transfer-auto-v1')
if root.exists():
    raise ValueError('new_readiness_output_required')
rclpy.init()
node = rclpy.create_node('check_no_previous_restart_anchor')
buf = Buffer()
listener = TransformListener(buf, node)
end = time.monotonic() + 3
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=.03)
exists = buf.can_transform('map', 'odom', Time())
node.destroy_node()
rclpy.shutdown()
if exists:
    raise ValueError('existing_map_anchor_requires_review_no_duplicate')
cfg = json.loads((code/'config/apple_transfer_auto_v1.json').read_text())
env = yaml.safe_load(Path(cfg['environment']).read_text())
initial = env['initial_pose']
if env['internal_name'] != 'LayoutA':
    raise ValueError('wrong_layout')
root.mkdir()
procenv = dict(os.environ, ROS_LOG_DIR=str(root/'ros-logs'))
cmd = ['ros2', 'run', 'handyman_rebuild_ros2', 'handyman_initial_pose_localizer',
       '--ros-args', '-p', 'map_frame:=map', '-p', 'odom_frame:=odom',
       '-p', 'base_frame:=base_footprint']
for key in ('x', 'y', 'yaw'):
    cmd += ['-p', 'initial_pose.'+key+':='+str(float(initial[key]))]
with (root/'initial-localizer.log').open('x') as log:
    localizer = subprocess.Popen(cmd, env=procenv, stdin=subprocess.DEVNULL,
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
state = dict(initial_localizer_launcher_pid=localizer.pid, command=cmd,
             expected_initial_pose=initial, user_confirmed_initial_scene=True,
             old_numeric_anchor_reused=False, robot_commands_sent=False)
(root/'startup.json').write_text(json.dumps(state, indent=2))
time.sleep(2)
commands = {
    'ready': ['/usr/bin/python3', str(code/'inspect_transfer_ready.py'), '--output', str(root/'ready.json')],
    'cameras': ['/usr/bin/python3', str(code/'capture_grasp_scene.py'), str(root/'cameras'),
                '--hand-geometry', '--map-geometry', '--tracking-epoch', 'initial-readiness-20261018-01',
                '--duration', '7', '--interval', '1'],
    'scan': ['/usr/bin/python3', '/home/crazylearner/RoboCup-Japan-Open-2026/build/handyman-competition/runtime/source-table-boundary-audit-20261017-01/capture_table_localization_audit.py',
             '--output', str(root/'scans.json')],
    'inputs': ['bash', str(code/'run_apple_transfer_auto.sh'), '--check-inputs'],
}
children = {}
for name, command in commands.items():
    with (root/(name+'.log')).open('x') as log:
        children[name] = subprocess.Popen(command, env=procenv, stdin=subprocess.DEVNULL,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
codes = {name: p.wait(timeout=35) for name, p in children.items()}
state.update(readonly_process_exit_codes=codes, localizer_running=localizer.poll() is None)
(root/'completion.json').write_text(json.dumps(state, indent=2))
print(json.dumps(state), flush=True)
if any(codes.values()) or not state['localizer_running']:
    raise SystemExit(1)
