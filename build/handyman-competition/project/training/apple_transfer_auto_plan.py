"""Measured plans for the reviewed apple transfer. No ROS or motion authority."""
import hashlib
import math
from pathlib import Path

import numpy as np

from hand_camera_ray import rotation
from inverse_table_place_plan import OFFSET, release_height
from tabletop_support import (EDGE_GAP_M, EDGE_UNCERTAINTY_M, PLANNING_RESERVE_M,
                              TableSupport, build_support)

RELEASE_XY_TOLERANCE_M = .012


def release_region(measurement, snapshot, plan):
    """Accept actual support/clearance, not a millimetre match to planned XY.

The support raster contains only observed table-plane cells; occupied and unknown
areas remain holes. Thus an edge-safe object footprint also stays in the known
clear support region. This does not claim dynamic-obstacle prediction.
"""
    from tabletop_support import support_from_plan
    from task_outcome_policy import RELEASE_BOTTOM_GAP_M
    mapped = map_point(snapshot, np.asarray(measurement['center_base'], float))
    edge = support_from_plan(plan).query(mapped[:2], measurement['radius_m'])
    height = float(np.r_[mapped[:2], 1.] @ plan['plane_map'])
    gap = float(mapped[2] - measurement['radius_m'] - height)
    if not np.isfinite([*mapped, height, gap]).all():
        raise ValueError('invalid_measured_release_geometry')
    return dict(ready=bool(edge['safe'] and RELEASE_BOTTOM_GAP_M[0] <= gap <= RELEASE_BOTTOM_GAP_M[1]),
                center_map=mapped.tolist(), table_edge_safety=edge, estimated_bottom_gap_m=gap,
                planned_xy_error_m=float(np.linalg.norm(mapped[:2]-plan['release_xy_map_m'])),
                planned_xy_is_success_gate=False)


def validate_config(config):
    from motion_speed import load_profile
    load_profile(config.get('motion_profile', 'baseline'))
    load_profile(config.get('navigation_motion_profile',config.get('motion_profile','baseline')))
    if (config.get('version') != 1 or config.get('layout') != 'LayoutA'
            or config.get('profile') != 'layouta_bedroom_apple_to_square_table'
            or config.get('task') != dict(object='apple', source_room='bedroom', destination='square_low_table')
            or config.get('source_table_id') != 'white_side_table#bedroom_1'
            or config.get('destination_id') != 'square_low_table#living_room'
            or config.get('destination_room') != 'living_room'):
        raise ValueError('unsupported_transfer_profile')
    if config.get('navigation_tolerances') != [.06, .08, .06]:
        raise ValueError('navigation_parameters_must_remain_verified_values')
    if config.get('execution') != dict(auto_protocol_success=False, automatic_action_retry=False):
        raise ValueError('automatic_retry_or_protocol_success_not_supported')
    p = config['policy']
    if type(p.get('efficient_non_navigation', False)) is not bool:
        raise ValueError('invalid_non_navigation_efficiency_option')
    if p.get('placement_approach_mode','segmented') not in ('single','segmented'):
        raise ValueError('invalid_placement_approach_mode')
    if type(p.get('near_grasp_roi_confirmation',False)) is not bool:
        raise ValueError('invalid_near_grasp_roi_option')
    if type(p.get('near_grasp_geometry_handoff',False)) is not bool:
        raise ValueError('invalid_near_grasp_geometry_option')
    if p.get('near_grasp_geometry_handoff',False):
        from near_apple_geometry import check_gap
        check_gap(p.get('near_grasp_geometry_gap_m'))
    if type(p.get('post_lift_rgbd_follow_verification',False)) is not bool:
        raise ValueError('invalid_post_lift_rgbd_follow_option')
    for key in ('source_staging_goal', 'destination_staging_goal'):
        if not isinstance(config.get(key), (list, tuple)) or len(config[key]) != 3:
            raise ValueError('invalid_staging_pose')
    numeric = [*config['source_staging_goal'], *config['destination_staging_goal'],
               *p['grasp_lift_range_m'], p['release_gap_m'], p['patch_radius_m'],
               p['edge_margin_m'], p['edge_uncertainty_m'],
               p['retreat_m'], p['observer_duration_s']]
    if not all(type(x) in (int, float) and math.isfinite(x) for x in numeric):
        raise ValueError('invalid_numeric_configuration')
    if (p['grasp_lift_range_m'] != [.24, .30]
            or not .005 <= p['release_gap_m'] <= .03
            or p['patch_radius_m'] != .09 or p['edge_margin_m'] != EDGE_GAP_M
            or p['edge_uncertainty_m'] != EDGE_UNCERTAINTY_M
            or 'max_placement_step_m' not in p
            or p['max_placement_step_m'] != (None if p.get('placement_approach_mode','segmented')=='single' else .4) or p['retreat_m'] != .25
            or not 180 <= p['observer_duration_s'] <= 600):
        raise ValueError('outside_reviewed_transfer_policy')
    return config


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def footprint(destination):
    bounds = destination['axis_aligned_bounds']
    center = np.array([bounds['center']['x'], bounds['center']['y']], float)
    half = np.array([bounds['size']['x'], bounds['size']['y']], float) / 2
    if not np.isfinite([center, half]).all() or np.any(half <= 0):
        raise ValueError('invalid_furniture_footprint')
    return center, half


def map_pose(snapshot):
    tf = snapshot['tracking_transforms']['map_from_base_at_depth']
    if tf['stamp_ns'] != snapshot['streams']['depth']['stamp_ns']:
        raise ValueError('map_transform_not_image_time')
    if tf.get('source_frame') != 'base_link' or tf.get('target_frame') != 'map':
        raise ValueError('invalid_map_transform')
    r = rotation(tf['quaternion'])
    return [*tf['translation'][:2], float(np.arctan2(r[1, 0], r[0, 0]))]


def map_point(snapshot, point):
    map_pose(snapshot)
    tf = snapshot['tracking_transforms']['map_from_base_at_depth']
    return rotation(tf['quaternion']) @ point + tf['translation']


def fresh_snapshot(snapshot, now_ns, epoch=None):
    if epoch is not None and snapshot.get('tracking_epoch') != epoch:
        raise ValueError('observation_epoch_mismatch')
    stamps = [snapshot['streams'][k]['stamp_ns'] for k in ('rgb', 'depth')]
    if (any(type(s) is not int or s <= 0 or not 0 <= (now_ns - s) / 1e9 <= .4 for s in stamps)
            or abs(stamps[0] - stamps[1]) > 30_000_000):
        raise ValueError('fresh_synchronised_rgbd_required')
    if not -.02 <= (now_ns - snapshot['joint_stamp_ns']) / 1e9 <= .4:
        raise ValueError('fresh_joint_snapshot_required')
    map_pose(snapshot)


def plan_grasp(observation, snapshot, source_table, evidence, config):
    identity = observation['identity_tracking']
    if identity.get('identity_confirmed') is not True:
        raise ValueError('confirmed_identity_required')
    proposals = observation.get('tracking_proposals', [])
    if len(proposals) != 1 or 'rejected_reason' in proposals[0]:
        raise ValueError('unique_measured_apple_required')
    proposal = proposals[0]
    support = proposal['support']
    if (proposal['detection']['name'] != 'apple' or support.get('support_consistent') is not True
            or proposal['candidate']['stamp_ns'] != evidence['stamp_ns']):
        raise ValueError('same_frame_measured_support_required')
    target = np.array(evidence['target_base_m'], float)
    mapped = map_point(snapshot, target)
    center, half = footprint(source_table)
    radius = proposal['candidate']['radius_m']
    if target.shape != (3,) or not np.isfinite([*target, radius]).all() or not .015 <= radius <= .065:
        raise ValueError('invalid_measured_apple_geometry')
    if np.any(half - abs(mapped[:2] - center) <= radius + .01):
        raise ValueError('apple_not_on_requested_source_table')
    # Same measured support/top midpoint used in the successful manual trial.
    palm_height = support['support_top_midpoint_z_m']
    lift = palm_height - OFFSET
    if (not math.isfinite(lift) or not config['policy']['grasp_lift_range_m'][0] <= lift <= config['policy']['grasp_lift_range_m'][1]
            or abs(target[2] - palm_height) > .025):
        raise ValueError('measured_grasp_height_outside_reviewed_range')
    return dict(contact_start_lift_m=lift, palm_height_m=palm_height,
                measured_target_base_m=target.tolist(), measured_target_map_m=mapped.tolist(),
                source_depth_stamp_ns=evidence['stamp_ns'], source_table_id=source_table['id'],
                motion_authorized=False)


def depth_in_map(depth, snapshot):
    if abs(snapshot['streams']['rgb']['stamp_ns'] - snapshot['streams']['depth']['stamp_ns']) > 30_000_000:
        raise ValueError('unsynchronised_rgbd')
    map_pose(snapshot)
    k = np.asarray(snapshot['rgb_k']).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('invalid_intrinsics')
    v, u = np.indices(depth.shape)
    valid = np.isfinite(depth) & (depth > .05) & (depth < 4)
    z = depth[valid]
    tf = snapshot['base_from_rgb']
    base = np.c_[(u[valid] - k[0, 2]) * z / k[0, 0], (v[valid] - k[1, 2]) * z / k[1, 1], z]
    base = base @ rotation(tf['quaternion']).T + tf['translation']
    mapped = snapshot['tracking_transforms']['map_from_base_at_depth']
    return base @ rotation(mapped['quaternion']).T + mapped['translation']


def clear_patch(points, xy, plane, radius):
    delta = points[:, :2] - xy
    inside = np.linalg.norm(delta, axis=1) <= radius
    sample = points[inside]
    heights = sample[:, 2] - np.c_[sample[:, :2], np.ones(len(sample))] @ plane
    table = sample[abs(heights) <= .006]
    obstacles = int(np.count_nonzero((heights > .015) & (heights < .25)))
    # Require visible table in all eight sectors; unknown space is not a clear patch.
    sectors = ((np.arctan2(table[:, 1] - xy[1], table[:, 0] - xy[0]) + math.pi) * 4 / math.pi).astype(int) % 8
    visible = len(table) >= 150 and all(np.count_nonzero(sectors == i) >= 3 for i in range(8))
    return dict(clear=bool(visible and obstacles == 0), table_surface_pixels=len(table),
                obstacle_pixels=obstacles, all_eight_sectors_observed=bool(visible), radius_m=radius)


def plan_placement(depth, snapshot, table, plane, held, config, task_id, *, max_support_radius_m=.065):
    if (len(held['center_minus_palm_m']) != 3 or not np.isfinite([*held['center_minus_palm_m'], held['radius_m'], held['holding_opening_rad']]).all()
            or max_support_radius_m not in (.065,.09)
            or not .015 <= held['radius_m'] <= max_support_radius_m or not .02 <= held['holding_opening_rad'] <= .62):
        raise ValueError('invalid_verified_held_geometry')
    center, half = footprint(table)
    # Use the current exported footprint and actual observed support, not an
    # equality test against one historical table's global centre/dimensions.
    # The reviewed approach direction, task/destination and reach checks remain.
    p = config['policy']
    points = depth_in_map(depth, snapshot)
    coef = np.array(plane['plane_map'])
    if plane['destination_id'] != table['id'] or table['room'] != config['destination_room']:
        raise ValueError('wrong_destination_plane')
    if plane['source_depth_stamp_ns'] != snapshot['streams']['depth']['stamp_ns']:
        raise ValueError('plane_capture_mismatch')
    support_model = build_support(points, center, half, coef, table['id'], plane['source_depth_stamp_ns'])
    support = TableSupport(support_model, table['id'])
    positions = support.safe_centers(held['radius_m'], PLANNING_RESERVE_M)
    # Safety is a pass/fail constraint, NOT a reward for going ever deeper.
    # Compare finite approaches from the measured base and horizontal held pose.
    from pregrasp_geometry import expected_horizontal_center
    rotation_map = rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
    reachable = []
    for xy in positions:
        height = float(np.r_[xy, 1.] @ coef)
        palm_height = release_height(height, held['bottom_below_palm_m'], p['release_gap_m'])
        predicted = map_point(snapshot, expected_horizontal_center(palm_height, held['center_minus_palm_m']))
        delta = (rotation_map.T @ np.r_[xy - predicted[:2], 0.])[:2]
        distance = float(np.linalg.norm(delta))
        # A single measured placement uses its FULL finite translation. No
        # fixed metre cap; legacy segmented callers retain their old envelope.
        if delta[0] > 0 and math.isfinite(distance) and (p.get('placement_approach_mode','segmented')=='single' or distance <= .6):
            reachable.append((distance, xy, palm_height, height))
    reachable.sort(key=lambda item: (item[0], item[1][0], item[1][1]))
    candidates = []
    for distance, xy, palm_height, height in reachable:
        patch = clear_patch(points, xy, coef, max(p['patch_radius_m'],held['radius_m']+.01) if max_support_radius_m>.065 else p['patch_radius_m'])
        if len(candidates) < 32:
            candidates.append(dict(xy_map_m=xy.tolist(), approach_distance_m=distance, **patch))
        if patch['clear']:
            return dict(task_id=task_id, destination_id=table['id'], destination_room=table['room'],
                        release_xy_map_m=xy.tolist(), palm_height_m=palm_height,
                        plane_map=coef.tolist(), table_height_at_release_m=height,
                        footprint_center=center.tolist(), footprint_half_size=half.tolist(),
                        held_center_minus_palm_m=held['center_minus_palm_m'], holding_opening_rad=held['holding_opening_rad'],
                        radius_m=held['radius_m'], patch=patch, checked_candidates=candidates,
                        table_edge_policy=dict(version=1, minimum_outer_gap_m=EDGE_GAP_M,
                            uncertainty_allowance_m=EDGE_UNCERTAINTY_M,
                            selection='shortest_reachable_approach_after_clearance_passes'),
                        observed_table_support=support_model,
                        table_edge_safety=support.query(xy, held['radius_m']),
                        planned_approach_distance_m=distance,
                        planning_alignment_and_radius_reserve_m=PLANNING_RESERVE_M,
                        safe_reachable_candidate_count=len(reachable),
                        source_depth_stamp_ns=plane['source_depth_stamp_ns'], motion_authorized=False)
    raise ValueError('no_reachable_clear_patch_with_table_edge_margin')


def placement_delta(measurement, snapshot, plan):
    r = rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
    mapped = map_point(snapshot, np.array(measurement['center_base']))
    delta = (r.T @ np.r_[np.array(plan['release_xy_map_m']) - mapped[:2], 0.])[:2]
    if not np.isfinite(delta).all() or not .001 < np.linalg.norm(delta) <= .4 or delta[0] <= 0:
        raise ValueError('placement_requires_unreviewed_reposition')
    return [*delta.tolist(), 0.]


def placement_step(measurement, snapshot, plan, *, single_move=False):
    """Full measured approach when explicitly selected; legacy segments remain opt-in."""
    if type(single_move) is not bool:raise ValueError('invalid_single_move_option')
    target=np.asarray(plan['release_xy_map_m'],float)
    if target.shape!=(2,) or not np.isfinite(target).all():raise ValueError('invalid_release_target')
    r = rotation(snapshot['tracking_transforms']['map_from_base_at_depth']['quaternion'])
    mapped = map_point(snapshot, np.array(measurement['center_base']))
    delta = (r.T @ np.r_[target - mapped[:2], 0.])[:2]
    distance = float(np.linalg.norm(delta))
    if not np.isfinite(delta).all() or not math.isfinite(distance) or (not single_move and distance > .6):
        raise ValueError('placement_requires_unreviewed_reposition')
    if 'observed_table_support' in plan:
        region = release_region(measurement, snapshot, plan)
        if region['ready']:
            return dict(aligned=True, distance_m=distance, delta_m=[0.,0.,0.], release_region=region)
        # At the planned XY but not above safe support/height: no horizontal
        # correction can fix this. Do not declare an aligned endpoint.
        if distance <= RELEASE_XY_TOLERANCE_M:
            raise ValueError('actual_release_region_not_ready')
    elif distance <= RELEASE_XY_TOLERANCE_M:  # Legacy plans only; live release requires support.
        return dict(aligned=True, distance_m=distance, delta_m=[0.,0.,0.])
    if delta[0] <= 0:
        raise ValueError('placement_requires_unreviewed_reposition')
    if not single_move and distance > .35:
        delta *= .32 / distance
    return dict(aligned=False, distance_m=distance, delta_m=[*delta.tolist(),0.])
