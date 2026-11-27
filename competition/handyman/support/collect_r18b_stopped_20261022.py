"""Read-only terminal capture; no motion or automatic continuation."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

BASE = Path('/__HANDYMAN_PACKAGE__/runtime')
CODE = Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1')
TRIAL = BASE / 'automatic-apple-r18b-20261022-01'
OUT = BASE / 'apple-r18b-post-stopped-20261022-01'
sys.path.insert(0, str(CODE))
from apple_transfer_automation import RobotLock, write_json
from apple_transfer_auto_plan import map_pose

read = lambda p: json.loads(p.read_text())
result = read(TRIAL / 'result.json')
if result['status'] != 'stopped':
    raise ValueError('stopped_trial_required')
with RobotLock():
    OUT.mkdir(exist_ok=False)
    with (OUT / 'capture.log').open('x') as stream:
        child = subprocess.run([sys.executable, str(CODE / 'capture_grasp_scene.py'), str(OUT / 'cameras'),
                                '--hand-geometry', '--map-geometry', '--tracking-epoch',
                                'r18b-post-stopped-20261022-01', '--duration', '7', '--interval', '.6',
                                '--stop-after-frames', '1'], stdout=stream, stderr=subprocess.STDOUT, timeout=25)
    if child.returncode:
        raise ValueError('terminal_capture_failed')
    capture = sorted((OUT / 'cameras').glob('*/snapshot.json'))[-1].parent
    snapshot = read(capture / 'snapshot.json')
    import rclpy
    rclpy.init()
    node = rclpy.create_node('r18b_terminal_command_graph_readonly')
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
    topics = ['/hsrb/' + tail for tail in ('command_velocity', 'omni_base_controller/command',
              'arm_trajectory_controller/command', 'gripper_controller/command', 'head_trajectory_controller/command')]
    pubs = {t: node.count_publishers(t) for t in topics}
    node.destroy_node()
    rclpy.shutdown()
    pose = map_pose(snapshot)
    retract = read(TRIAL / 'carry-retract/result.json')
    anchor = retract['base_anchor']
    events = [json.loads(line) for line in (TRIAL / 'journal.jsonl').read_text().splitlines()]
    before = read(TRIAL / 'before-carry-retract/000000/snapshot.json')
    observations = sorted((TRIAL / 'source-observe').glob('*/snapshot.json'))
    trajectory = []
    for path in observations:
        s = read(path)
        stamp = s['streams']['depth']['stamp_ns']
        if stamp < events[-4]['time_ns'] - 4_000_000_000:
            continue
        try:
            p = map_pose(s)
        except Exception:
            continue
        trajectory.append(dict(stamp_ns=stamp, base_map_pose=p,
                               translation_from_retract_anchor_m=math.dist(p[:2], anchor[:2]),
                               yaw_from_retract_anchor_rad=math.atan2(math.sin(p[2]-anchor[2]), math.cos(p[2]-anchor[2])),
                               joints=s['joints'], evidence=str(path)))
    report = dict(status='stopped_trial_current_state_collected', trial_result=result,
                  duration_to_stop_s=(events[-1]['time_ns']-events[0]['time_ns'])/1e9,
                  full_transfer_completed=False, full_transfer_duration_s=None,
                  current_capture=str(capture), current_base_map_pose=pose, current_joints=snapshot['joints'],
                  command_publishers=pubs, base_before_retract=map_pose(before), retract_anchor=anchor,
                  current_translation_from_retract_anchor_m=math.dist(pose[:2], anchor[:2]),
                  current_yaw_from_retract_anchor_rad=math.atan2(math.sin(pose[2]-anchor[2]), math.cos(pose[2]-anchor[2])),
                  retract_checks=dict(max_base_translation_m=.015, max_base_yaw_rad=.03),
                  source_observer_tail=trajectory[-20:],
                  terminal_corrections=read(TRIAL / 'grasp/result.json')['terminal_alignment']['corrections'],
                  independent_lift_verification=read(TRIAL / 'grasp/close-lift/independent-lift-verification.json'),
                  robot_commands_sent_during_audit=False, automatic_retry=False,
                  navigation_or_production_code_changed=False)
    write_json(OUT / 'result.json', report)
    small = {k: v for k, v in report.items() if k not in ('source_observer_tail', 'independent_lift_verification')}
    small['recorded_retract_tail'] = [{k: v for k, v in x.items() if k not in ('joints',)} for x in trajectory[-8:]]
    print(json.dumps(small), flush=True)
