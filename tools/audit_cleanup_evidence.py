#!/usr/bin/env python3
"""Inspect saved sensor evidence without ROS, inference or robot commands.

This reports observations, not task correctness: the recording contains no
moderator ground truth and only the last three frames of each enabled phase.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def inspect(path):
    with np.load(path, allow_pickle=False) as data:
        pointing = json.loads(str(data['pointing_json']))
        matrix = data['camera_to_odom']
        result = {
            'file': path.name,
            'rgb_depth_delta_ms': round(
                abs(int(data['rgb_stamp_ns']) - int(data['depth_stamp_ns'])) / 1e6, 1),
            'saved_tf_available': bool(np.isfinite(matrix).all()),
            'person_visible': pointing['person_is_valid'],
            'gesture_visible': pointing['is_valid'],
            'plane_valid': pointing['plane_valid'],
            'measured_direction': pointing['depth_direction_valid'],
        }
        if pointing['is_valid']:
            start = np.array([pointing['wrist_pixel_x'], pointing['wrist_pixel_y']])
            end = np.array([pointing['point_pixel_x'], pointing['point_pixel_y']])
            height, width = data['rgb_bgr'].shape[:2]
            result['image_segment_px'] = round(float(np.linalg.norm(end - start)), 1)
            result['endpoint_in_image'] = bool(
                0 <= end[0] < width and 0 <= end[1] < height)
            result['image_segment_start_end'] = [start.tolist(), end.tolist()]
            result['direction_odom'] = pointing['direction']
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    files = sorted(args.directory.glob('*.npz'))
    if not files:
        parser.error('no evidence files found')
    for path in files:
        print(json.dumps(inspect(path), ensure_ascii=False))


if __name__ == '__main__':
    main()
