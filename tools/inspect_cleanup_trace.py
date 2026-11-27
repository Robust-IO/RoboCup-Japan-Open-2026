#!/usr/bin/env python3
"""Inspect per-frame traces without ROS. This does not replay the controller.

Class position changes are warnings, not instance tracking: two objects of the
same class can legitimately be at different positions. Last torn JSONL records
are reported and skipped so an interrupted run remains inspectable.
"""
import argparse
import json
import math
from collections import Counter
from pathlib import Path


def inspect(path):
    counts = Counter()
    previous = {}
    last = None
    events = []
    with open(path, encoding='utf-8') as stream:
        for line_no, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                events.append({'line': line_no, 'reason': 'invalid_json'})
                continue
            counts['frames'] += 1
            phase = row['phase']
            counts[f'phase_{phase}_frames'] += 1
            if last is not None and row['wall_ns'] < last:
                events.append({'line': line_no, 'reason': 'wall_clock_reversed',
                               'seconds': (last-row['wall_ns'])/1e9})
            last = row['wall_ns']
            counts['valid_gestures'] += bool(row['pointing']['is_valid'])
            counts['missing_tf'] += row['camera_to_odom'] is None
            audit = row.get('gesture_audit', {})
            counts['pose_' + audit.get('pose_status', 'unrecorded')] += 1
            for arm in audit.get('arms', []):
                counts['arm_' + arm['reason']] += 1
            if 'close_hand' in audit:
                counts['hand_' + audit['close_hand']['status']] += 1
            for obj, sample in zip(row['objects']['objects'], row['depth_samples']):
                if not obj['has_3d_position']:
                    continue
                key = (phase, obj['class_name'])
                xyz = [obj['position_3d'][k] for k in ('x', 'y', 'z')]
                if key in previous and math.dist(previous[key], xyz) > 0.4:
                    events.append({'line': line_no, 'reason': 'class_position_change',
                                   'class': key[1], 'before': previous[key], 'after': xyz})
                previous[key] = xyz
                matrix, camera = row['camera_to_odom'], sample['camera_xyz']
                if matrix is not None and camera is not None:
                    transformed = [sum(matrix[i][j]*camera[j] for j in range(3)) +
                                   matrix[i][3] for i in range(3)]
                    if math.dist(transformed, xyz) > 0.001:
                        events.append({'line': line_no, 'reason': 'transform_mismatch',
                                       'class': key[1]})
    return {'counts': dict(counts), 'events': events}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path, help='observations.jsonl or its directory')
    args = parser.parse_args()
    path = args.path / 'observations.jsonl' if args.path.is_dir() else args.path
    print(json.dumps(inspect(path), ensure_ascii=False, indent=2))
