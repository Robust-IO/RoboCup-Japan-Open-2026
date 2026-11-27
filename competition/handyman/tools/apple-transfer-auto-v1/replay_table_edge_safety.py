"""Offline regression of the edge-unsafe recorded apple placement. No ROS/motion.

Replans from the original outside-table RGBD. The old release and final landing
are EXPECTED to fail the new edge test. New coordinates are plans, not live
execution results; historical timestamps are never reused as motion leases.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from apple_transfer_auto_plan import plan_placement, validate_config
from measure_destination_surface import measure
from tabletop_support import support_from_plan
from verify_table_placement import verify


def read(path):
    return json.loads(Path(path).read_text())


def replay(source, placement, config):
    old_plan = read(source / 'placement-plan.json')
    surface = read(source / 'destination-surface.json')
    paths = list((source / 'destination-outside').glob('*/snapshot.json'))
    capture = next((p.parent for p in paths if read(p)['streams']['depth']['stamp_ns']
                    == surface['source_depth_stamp_ns']), None)
    if capture is None:
        raise ValueError('recorded_surface_capture_missing')
    snapshot = read(capture / 'snapshot.json')
    depth = np.load(capture / 'depth_m.npy', allow_pickle=False)
    table = next(x for x in read(config['semantic_map'])['destinations']
                 if x['id'] == config['destination_id'])
    held = read(source / 'verified-held-geometry.json')
    plan = plan_placement(depth, snapshot, table, measure(depth, snapshot, table), held, config, 'edge-replay-only')
    support = support_from_plan(plan)
    release = read(placement / 'place-release/result.json')['release_geometry_evidence']
    old_release = support.query(release['center_map'][:2], release['measurement']['radius_m'])
    audit = read(placement / 'placed-final-audit.json')
    actions = {k: read(placement / f'place-{k}/result.json') for k in ('release', 'raise', 'retract')}
    verification = verify(audit, release, actions, config['destination_id'], config['destination_room'],
                          placement_plan=plan)
    final_edges = []
    for frame in audit['frames']:
        for apple in frame['apple_measurements']:
            if math.dist(apple['center_map'][:2], release['center_map'][:2]) < .1:
                final_edges.append(support.query(apple['center_map'][:2], apple['candidate']['radius_m']))
    if (not plan['table_edge_safety']['safe'] or old_release['safe'] or not final_edges
            or any(x['safe'] for x in final_edges) or verification['placement_success_verified']):
        raise ValueError('recorded_edge_regression_failed')
    return dict(status='recorded_table_edge_regression_passed', clock='HISTORICAL_REPLAY_ONLY_NOT_LIVE',
                input_capture=str(capture), old_release_xy_map_m=old_plan['release_xy_map_m'],
                proposed_release_xy_map_m=plan['release_xy_map_m'],
                proposed_move_from_old_plan_m=float(np.linalg.norm(np.array(plan['release_xy_map_m']) - old_plan['release_xy_map_m'])),
                proposed_approach_distance_m=plan['planned_approach_distance_m'],
                proposed_edge_safety=plan['table_edge_safety'], old_release_edge_safety=old_release,
                old_final_frame_edge_safety=final_edges, old_final_verification_with_new_policy=verification,
                placement_plan=plan, robot_commands_sent=False, unity_scene_modified=False,
                navigation_parameters_changed=False, automatic_live_transfer_verified=False,
                limitation='Rejects prior unsafe landing and finds a finite observed safe proposal. New approach/release and whole-run automation still require live validation.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--placement-run', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=Path(__file__).parent / 'config/apple_transfer_auto_v1.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('output_exists')
    result = replay(args.source_run, args.placement_run, validate_config(read(args.config)))
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in result.items() if k not in
                      ('placement_plan', 'old_final_verification_with_new_policy', 'old_final_frame_edge_safety')}, indent=2))


if __name__ == '__main__':
    main()
