"""New timed transfer, preceded by a fresh receive-only localization check."""
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

BASE = Path('/__HANDYMAN_PACKAGE__/runtime')
CODE = Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
CONTROL = BASE / 'apple-r18b-full-live-control-20261022-01'
OUTPUT = BASE / 'automatic-apple-r18b-20261022-01'
TASK = 'apple-r18b-full-live-20261022-01'
sys.path.insert(0, str(CODE))
from apple_transfer_automation import RobotLock, check_inputs, write_json
from hand_camera_ray import rotation
from grasp_terminal_correction import MAX_CORRECTIONS

if CONTROL.exists() or OUTPUT.exists():
    raise ValueError('new_trial_required_no_replay')
if os.environ.get('ROS_DOMAIN_ID') != '71' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise ValueError('domain71_localhost_required')
CONTROL.mkdir()
preparation_started_ns = time.time_ns()
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
hashes = {}
for name in ('apple-efficiency-r17-20261022-deployment.json',
             'apple-terminal-r18-20261022-deployment.json',
             'apple-terminal-r18b-20261022-deployment.json'):
    hashes.update(json.loads((BASE / name).read_text())['hashes'])
if any(sha(CODE / n) != d for n, d in hashes.items()):
    raise ValueError('deployed_code_changed')
baseline = json.loads((BASE / 'apple-terminal-r18-20261022-baseline.json').read_text())
if any(sha(CODE / n) != d for n, d in baseline['frozen']['runtime'].items()):
    raise ValueError('frozen_navigation_or_outcome_code_changed')
if MAX_CORRECTIONS != 5:
    raise ValueError('five_terminal_corrections_required')
cfg = json.loads((CODE / 'config/apple_transfer_auto_v1.json').read_text())
env = yaml.safe_load(Path(cfg['environment']).read_text())
initial = np.array([env['initial_pose'][k] for k in ('x', 'y', 'yaw')], float)
map_path = Path('/__HANDYMAN_PACKAGE__/project/src/handyman_rebuild_ros2/maps/LayoutA/map.yaml')
meta = yaml.safe_load(map_path.read_text())
grid = np.asarray(Image.open(map_path.parent / meta['image']))
if env['internal_name'] != 'LayoutA' or meta['origin'][2] != 0 or meta.get('negate', 0) != 0:
    raise ValueError('reviewed_layout_and_map_required')
occupied = (255 - grid.astype(float)) / 255 > meta['occupied_thresh']
dist = distance_transform_edt(~occupied) * meta['resolution']

with RobotLock():
    write_json(CONTROL / 'input-files-verified.json', check_inputs(cfg))
    rclpy.init()
    node = rclpy.create_node('r18b_new_trial_localization_audit')
    buf = Buffer()
    listener = TransformListener(buf, node)
    scans = deque(maxlen=40)
    sub = node.create_subscription(LaserScan, '/hsrb/base_scan', scans.append, qos_profile_sensor_data)
    rows = []
    seen = set()
    deadline = time.monotonic() + 12
    next_sample = time.monotonic() + 2
    try:
        while time.monotonic() < deadline and len(rows) < 3:
            rclpy.spin_once(node, timeout_sec=.02)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0)
            if time.monotonic() < next_sample:
                continue
            now = node.get_clock().now().nanoseconds
            for scan in reversed(scans):
                when = Time.from_msg(scan.header.stamp)
                stamp = when.nanoseconds
                frames = [('base_from_scan', 'base_footprint', scan.header.frame_id),
                          ('map_from_base', 'map', 'base_footprint'),
                          ('odom_from_base', 'odom', 'base_footprint')]
                if stamp in seen or not 0 <= (now - stamp) / 1e9 <= .4:
                    continue
                if not all(buf.can_transform(t, s, when) for _, t, s in frames):
                    continue
                row = dict(stamp_ns=stamp, age_at_capture_s=(now - stamp) / 1e9,
                           angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                           ranges=[float(v) if math.isfinite(v) else None for v in scan.ranges])
                transforms = {}
                for key, target, source in frames:
                    tf = buf.lookup_transform(target, source, when).transform
                    q = tf.rotation
                    transforms[key] = dict(translation=[tf.translation.x, tf.translation.y, tf.translation.z],
                                           quaternion=[q.x, q.y, q.z, q.w])
                row.update(transforms)
                ranges = np.array([np.nan if v is None else v for v in row['ranges']])
                angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
                idx = np.flatnonzero(np.isfinite(ranges) & (ranges > max(.15, scan.range_min)) &
                                     (ranges < min(9., scan.range_max)))[::3]
                points = np.c_[ranges[idx] * np.cos(angles[idx]), ranges[idx] * np.sin(angles[idx]), np.zeros(len(idx))]
                tf = transforms['base_from_scan']
                points = points @ rotation(tf['quaternion']).T + tf['translation']
                tf = transforms['map_from_base']
                mapped = (points @ rotation(tf['quaternion']).T + tf['translation'])[:, :2]
                cells = np.floor((mapped - meta['origin'][:2]) / meta['resolution']).astype(int)
                u = cells[:, 0]
                v = grid.shape[0] - 1 - cells[:, 1]
                valid = (u >= 0) & (u < grid.shape[1]) & (v >= 0) & (v < grid.shape[0])
                errors = np.full(len(u), 10.)
                errors[valid] = dist[v[valid], u[valid]]
                r = rotation(tf['quaternion'])
                pose = [*tf['translation'][:2], math.atan2(r[1, 0], r[0, 0])]
                row.update(base_map_pose=pose, points=len(errors),
                           inliers_within10cm=float(np.mean(errors <= .1)),
                           median_error_m=float(np.median(errors)))
                rows.append(row)
                seen.add(stamp)
                next_sample = time.monotonic() + .3
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    audit = dict(configured_initial_pose=initial.tolist(), scans=rows, robot_commands_sent=False,
                 existing_anchor_revalidated_against_fresh_laser=True)
    write_json(CONTROL / 'fresh-localization-audit.json', audit)
    if len(rows) != 3 or any(r['points'] < 60 or r['inliers_within10cm'] < .9 or
                             r['median_error_m'] > .05 or math.dist(r['base_map_pose'][:2], initial[:2]) > .10 or
                             abs(math.atan2(math.sin(r['base_map_pose'][2] - initial[2]),
                                            math.cos(r['base_map_pose'][2] - initial[2]))) > .10 for r in rows):
        raise ValueError('fresh_scene_localization_not_verified_no_motion_started')
    with (CONTROL / 'readiness.log').open('x') as stream:
        result = subprocess.run([sys.executable, str(CODE / 'inspect_transfer_ready.py'),
                                 '--efficient-transfer', '--output', str(CONTROL / 'ready.json')],
                                stdout=stream, stderr=subprocess.STDOUT, timeout=25)
    if result.returncode:
        raise ValueError('fresh_readiness_failed_no_motion_started')
    ready = json.loads((CONTROL / 'ready.json').read_text())
    if not ready['ready']:
        raise ValueError('fresh_empty_tucked_start_required')

# Release preparation lock before automation acquires its own.
command = [sys.executable, str(CODE / 'apple_transfer_automation.py'), '--run',
           '--confirm-layout', 'LayoutA', '--confirm-localization', '--simulation-ignore-collision',
           '--task-id', TASK, '--output', str(OUTPUT)]
execution_started_ns = time.time_ns()
with (CONTROL / 'automation.log').open('x') as stream:
    child = subprocess.Popen(command, env=dict(os.environ, ROS_LOG_DIR=str(CONTROL / 'ros-logs')),
                             stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
                             start_new_session=True)
report = dict(status='new_full_timed_transfer_started', pid=child.pid, task_id=TASK, output=str(OUTPUT),
              revision='terminal-correction-r18b', max_terminal_corrections=MAX_CORRECTIONS,
              command=command, preparation_started_ns=preparation_started_ns,
              execution_started_ns=execution_started_ns,
              preparation_elapsed_s=(execution_started_ns - preparation_started_ns) / 1e9,
              robot_start_pose=ready['base_map_pose'], navigation_unchanged=True,
              old_actions_replayed=False, automatic_retry=False,
              initial_scene_laser_verified=True, task_finished_protocol_enabled=False,
              log=str(CONTROL / 'automation.log'),
              timer_basis='Journal begin through verified_complete or stopped; all internal waits/corrections included')
write_json(CONTROL / 'startup.json', report)
print(json.dumps(report), flush=True)
