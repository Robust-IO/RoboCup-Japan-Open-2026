"""Convert a detection center to a ray, never to a measured 3-D target."""
import argparse
import json
from pathlib import Path
import numpy as np

# Validated against off-axis apple and 7 independent background features on
# 2026-10-10. Valid for this captured renderer configuration, not every Unity build.
HAND_RASTER_TO_FRAME = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])


def rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-3:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rays(snapshot, box, profile='unverified_standard'):
    box = np.asarray(box, dtype=float)
    size = snapshot['streams']['hand']
    if box.shape != (4,) or not np.isfinite(box).all():
        raise ValueError('Invalid bbox')
    x1, y1, x2, y2 = box
    if not (0 <= x1 < x2 <= size['width'] and 0 <= y1 < y2 <= size['height']):
        raise ValueError('BBox outside image')
    if np.any(np.asarray(snapshot['hand_d']) != 0):
        raise ValueError('Nonzero distortion requires undistortion')
    k = np.asarray(snapshot['hand_k']).reshape(3, 3)
    direction = np.linalg.solve(k, [(x1+x2)/2, (y1+y2)/2, 1])
    if profile == 'sigverse_hsr_raw_hand':
        raise ValueError('Deprecated wrong vertical sign; use sigverse_hsr_hand_v2 after configuration validation')
    if profile == 'sigverse_hsr_hand_v2':
        if size['frame'] != 'hand_camera_rgb_frame':
            raise ValueError('Unexpected hand frame for SIGVerse profile')
        direction = HAND_RASTER_TO_FRAME @ direction
    elif profile != 'unverified_standard':
        raise ValueError('Unknown raster profile')
    direction /= np.linalg.norm(direction)
    result = {'pixel_center': [(x1+x2)/2, (y1+y2)/2],
              'image_stamp_ns': size['stamp_ns'], 'motion_authorized': False,
              'optical_frame_alignment_verified': False,
              'raster_profile': profile,
              'profile_status': 'V2 checked on off-axis target and independent background features; multi-pose validation still incomplete.',
              'limitation': 'BBox-center ray only; no depth or grasp center measured.'}
    for frame in ('base', 'palm'):
        transform = snapshot[frame+'_from_hand']
        result[frame+'_ray'] = {'origin': transform['translation'],
                               'unit_direction': (rotation(transform['quaternion']) @ direction).tolist()}
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--bbox', nargs=4, type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=['unverified_standard', 'sigverse_hsr_raw_hand', 'sigverse_hsr_hand_v2'], default='unverified_standard')
    args = parser.parse_args()
    result = rays(json.loads(args.snapshot.read_text()), args.bbox, args.profile)
    with args.output.open('x') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))
