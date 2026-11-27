"""Fresh held-apple release evidence, checked at the actual hand publish boundary."""
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

from apple_transfer_auto_plan import fresh_snapshot, map_pose, release_region
from tracked_apple_surface import measure
from verify_apple_lift_sequence import palm_in_base
from tabletop_support import support_from_plan
from task_outcome_policy import MAX_HOLD_OPENING_DRIFT_RAD, pose_matches


def validate(capture, plan, *, now_ns, epoch, task_id, current_map_pose, current_joints):
    capture = Path(capture)
    if plan.get('task_id') != task_id or plan.get('destination_id') != 'square_low_table#living_room':
        raise ValueError('release_task_or_destination_mismatch')
    support_from_plan(plan)  # Legacy AABB-only plans must be replanned.
    s = json.loads((capture / 'snapshot.json').read_text())
    fresh_snapshot(s, now_ns, epoch)
    pose = map_pose(s)
    da = math.atan2(math.sin(pose[2] - current_map_pose[2]), math.cos(pose[2] - current_map_pose[2]))
    if math.dist(pose[:2], current_map_pose[:2]) > .01 or abs(da) > .02:
        raise ValueError('base_moved_since_release_observation')
    names = ('arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint', 'hand_motor_joint')
    observed_joints = [s['joints'][k] for k in names]
    if (not pose_matches(current_joints, observed_joints)
            or not .02 <= current_joints[5] <= .62
            or abs(current_joints[5]-plan['holding_opening_rad']) > MAX_HOLD_OPENING_DRIFT_RAD):
        raise ValueError('release_arm_or_holding_pose_changed')
    expected_center = palm_in_base(s) + np.asarray(plan['held_center_minus_palm_m'])
    measured = measure(np.asarray(Image.open(capture / 'rgb.png').convert('RGB')),
                       np.load(capture / 'depth_m.npy', allow_pickle=False), s, expected_center)
    region = release_region(measured, s, plan)
    # The required observed support model already verifies this table's actual
    # edge with the current explicit3cm+1cm allowance. Do not veto it a second time
    # using a furniture AABB that need not be the tabletop's physical edge.
    edge = region['table_edge_safety']
    if not edge['safe']:
        raise ValueError('apple_too_close_to_observed_table_edge')
    if not region['ready']:
        raise ValueError('release_height_not_verified')
    return dict(capture=str(capture), source_stamp_ns=s['streams']['depth']['stamp_ns'],
                **region,
                measurement=measured, task_id=task_id, motion_authorized=False)


def read_latest(recording, plan_path, **kwargs):
    from latest_capture_index import latest
    try:path=latest(recording,'snapshot.json')
    except FileNotFoundError:raise ValueError('no_current_release_capture') from None
    return validate(path.parent, json.loads(Path(plan_path).read_text()), **kwargs)
