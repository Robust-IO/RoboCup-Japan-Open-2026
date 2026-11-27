"""Track a confirmed penguin's beak in the calibrated hand camera.

RGB rigidity is evidence of a continuing grasp, not a new object depth sample.
Held 3D poses below are explicitly kinematic estimates anchored by preclose RGBD.
"""
import hashlib
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from canned_grasp_geometry import transform_point
from hand_camera_ray import HAND_RASTER_TO_FRAME, rotation
from penguin_geometry import NAME, surface_extent, validate_reference
from penguin_hold import expected_centers, palm_points, current_depth


def palm_at_capture(snapshot):
    bh, ph = snapshot['base_from_hand'], snapshot['palm_from_hand']
    r = rotation(bh['quaternion']) @ rotation(ph['quaternion']).T
    p = np.asarray(bh['translation']) - r @ ph['translation']
    if np.linalg.norm(p - snapshot['palm_base_at_depth']) > .005:
        raise ValueError('stationary_preclose_palm_transforms_required')
    return dict(translation=p.tolist(), quaternion=Rotation.from_matrix(r).as_quat().tolist())


def yellow_mask(bgr):
    return cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV),
                       np.array([15, 90, 50]), np.array([45, 255, 255]))


def create_reference(hand_bgr, snapshot, measurement, path, *, model):
    validate_reference(model)
    stream = snapshot['streams']['hand']
    stamp = stream['stamp_ns']
    if (measurement.get('class_name') != NAME or measurement.get('current_head_depth_measured') is not True
            or measurement['stamp_ns'] != snapshot['streams']['depth']['stamp_ns']
            or abs(stamp - measurement['stamp_ns']) > 100_000_000
            or stream['frame'] != 'hand_camera_rgb_frame'
            or hand_bgr.shape != (stream['height'], stream['width'], 3)
            or np.any(np.asarray(snapshot['hand_d']) != 0)):
        raise ValueError('current_calibrated_hand_and_measured_head_required')
    # Move the head from depth-time base coordinates to hand-image-time base.
    head_odom = transform_point(snapshot, measurement['cap_center_m'])
    hand_time = snapshot['tracking_transforms']['base_from_odom_at_hand']
    if (hand_time['stamp_ns'] != stamp or hand_time['source_frame'] != 'odom'
            or hand_time['target_frame'] != 'base_link'):
        raise ValueError('image_time_hand_transform_required')
    head = rotation(hand_time['quaternion']) @ head_odom + hand_time['translation']
    bh = snapshot['base_from_hand']
    frame = rotation(bh['quaternion']).T @ (head - bh['translation'])
    raster = HAND_RASTER_TO_FRAME.T @ frame
    radius = measurement['cap_radius_m']
    if raster[2] <= radius + .005:
        raise ValueError('head_projection_outside_hand_camera_model')
    k = np.asarray(snapshot['hand_k']).reshape(3, 3)
    pixel = (k @ raster)[:2] / raster[2]
    bound = 1.5 * radius * max(k[0, 0], k[1, 1]) / (raster[2] - radius)
    yy, xx = np.indices(hand_bgr.shape[:2])
    mask = yellow_mask(hand_bgr)
    mask[(xx - pixel[0])**2 + (yy - pixel[1])**2 > bound**2] = 0
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    candidates = [st for st in stats[1:] if 40 <= st[4] <= .6 * np.pi * bound**2]
    if len(candidates) != 1:
        raise ValueError('unique_penguin_beak_in_projected_head_region_required')
    x, y, w, h, count = map(int, candidates[0])
    box = [max(0, x-7), max(0, y-7), min(stream['width'], x+w+7), min(stream['height'], y+h+7)]
    x1, y1, x2, y2 = box
    patch = hand_bgr[y1:y2, x1:x2]
    path = Path(path)
    if path.exists():
        raise ValueError('new_hand_reference_path_required')
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), patch):
        raise ValueError('hand_reference_write_failed')
    return dict(class_name=NAME, template_path=str(path),
                template_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                box=box, reference_yellow_pixels=count,
                image_shape=list(hand_bgr.shape), seed_hand_stamp_ns=stamp,
                source_object_depth_stamp_ns=measurement['stamp_ns'],
                tracking_epoch=snapshot['tracking_epoch'], raster_profile='sigverse_hsr_hand_v2',
                head_projection_pixel=pixel.tolist(), identity_source='this_preclose_rgbd_verified_penguin',
                motion_authorized=False)


@lru_cache(maxsize=16)
def _template(path, digest):
    path = Path(path)
    if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
        raise ValueError('hand_reference_changed')
    image = cv2.imread(str(path))
    if image is None:
        raise ValueError('hand_reference_not_readable')
    return image


def verify_hand(hand_bgr, snapshot, reference, holding, *, now_ns=None):
    stamp = snapshot['streams']['hand']['stamp_ns']
    opening = snapshot['joints']['hand_motor_joint']
    if (reference.get('class_name') != NAME or snapshot.get('tracking_epoch') != reference['tracking_epoch']
            or stamp <= reference['seed_hand_stamp_ns'] or list(hand_bgr.shape) != reference['image_shape']
            or snapshot['streams']['hand']['frame'] != 'hand_camera_rgb_frame'
            or not .02 < holding <= .62 or abs(opening - holding) > .03):
        raise ValueError('same_penguin_current_hand_and_preserved_grip_required')
    if now_ns is not None and not -20_000_000 <= now_ns - stamp <= 400_000_000:
        raise ValueError('fresh_penguin_hand_image_required')
    template = _template(reference['template_path'], reference['template_sha256'])
    x1, y1, x2, y2 = reference['box']
    sx, sy = max(0, x1-32), max(0, y1-32)
    search = hand_bgr[sy:min(hand_bgr.shape[0], y2+32), sx:min(hand_bgr.shape[1], x2+32)]
    if min(search.shape[:2] - np.array(template.shape[:2])) < 0:
        raise ValueError('penguin_hand_search_region_invalid')
    _, score, _, where = cv2.minMaxLoc(cv2.matchTemplate(search, template, cv2.TM_CCOEFF_NORMED))
    x, y = sx+where[0], sy+where[1]
    patch = hand_bgr[y:y+template.shape[0], x:x+template.shape[1]]
    count = int(np.count_nonzero(yellow_mask(patch)))
    shift = [x-x1, y-y1]
    if score < .8 or count < .5 * reference['reference_yellow_pixels'] or np.linalg.norm(shift) > 24:
        raise ValueError('penguin_face_not_at_verified_hand_position')
    return dict(class_name=NAME, current_hand_rgb_verified=True, correlation=float(score),
                match_score_is_not_detection_probability=True, yellow_pixels=count, pixel_shift=shift,
                hand_stamp_ns=stamp, opening_rad=opening, measured_3d_object_position_available=False,
                new_target_identity_created=False, motion_authorized=False)


def contact_context(model, measurement, snapshot, hand_reference, holding):
    validate_reference(model)
    bp, hp = palm_points(measurement['center_m'], measurement['cap_center_m'], palm_at_capture(snapshot))
    return dict(class_name=NAME, grasp_lift_verified=False, holding_opening_rad=holding, model=model,
                body_palm_m=bp, cap_palm_m=hp, hand_face_reference=hand_reference,
                last_horizontal_measurement=measurement,
                body_minus_palm_horizontal_m=(np.asarray(measurement['center_m'])-snapshot['palm_base_at_depth']).tolist(),
                original_object_depth_stamp_ns=measurement['stamp_ns'],
                body_palm_source='preclose_rgbd_registered_shape_with_current_hand_rgb_rigidity',
                head_palm_source='preclose_rgbd_with_current_hand_rgb_rigidity')


def verify_lift(before, after, evidence, holding):
    if (before['tracking_epoch'] != after['tracking_epoch'] or evidence.get('current_hand_rgb_verified') is not True
            or evidence['hand_stamp_ns'] != after['streams']['hand']['stamp_ns']):
        raise ValueError('current_penguin_hand_lift_evidence_required')
    delta = transform_point(after, after['palm_base_at_depth']) - transform_point(before, before['palm_base_at_depth'])
    joint = after['joints']['arm_lift_joint'] - before['joints']['arm_lift_joint']
    if (min(delta[2], joint) < .03 or not .02 < holding <= .62
            or abs(after['joints']['hand_motor_joint']-holding) > .03):
        raise ValueError('penguin_hand_and_contact_did_not_lift_off_table')
    return dict(stamp_ns=after['streams']['depth']['stamp_ns'], grasp_lift_verified=True,
                measured_part='hand_rgb_rigidity_and_palm_motion', body_motion_independently_measured=False,
                object_depth_independently_measured=False, palm_delta_odom_m=delta.tolist(),
                hand_evidence=evidence, placement_verified=False)


def estimate_held(palm, snapshot, held, evidence):
    if (evidence.get('current_hand_rgb_verified') is not True
            or evidence['hand_stamp_ns'] != snapshot['streams']['hand']['stamp_ns']):
        raise ValueError('current_hand_proof_required_for_held_estimate')
    b, h = expected_centers(palm, held)
    model = held['model']
    shape = surface_extent(b, h, model['body_radius_m'], model['cap_radius_m'], model['body_above_support_m'])
    shape['horizontal_support_radius_m'] = max(shape['horizontal_support_radius_m'], model['observed_horizontal_extent_m'])
    return dict(class_name=NAME, center_m=b.tolist(), cap_center_m=h.tolist(), **shape,
                body_radius_m=model['body_radius_m'], cap_radius_m=model['cap_radius_m'],
                footprint_radius_m=shape['horizontal_support_radius_m'], height_m=model['height_m'],
                body_above_support_m=model['body_above_support_m'],
                stamp_ns=snapshot['streams']['depth']['stamp_ns'],
                stamp_semantics='current_kinematic_state_evaluation_not_object_depth_observation',
                original_object_depth_stamp_ns=held['original_object_depth_stamp_ns'],
                current_body_depth_measured=False, current_head_depth_measured=False,
                hidden_surfaces_measured=False, predicted_center_returned=True,
                position_source='preclose_rgbd_anchored_palm_kinematics_with_current_hand_rgb',
                motion_authorized=False)


def hold_evidence(sample, palm, held, *, now_ns=None, require_pose=False):
    """Accept either current hand appearance or independently measured depth."""
    snapshot = sample['snapshot']
    try:
        proof = verify_hand(sample['hand'], snapshot, held['hand_face_reference'],
                            held['holding_opening_rad'], now_ns=now_ns)
    except ValueError as exc:
        if str(exc) not in ('penguin_face_not_at_verified_hand_position', 'fresh_penguin_hand_image_required'):
            raise
        proof = current_depth(sample['rgb'], sample['depth'], snapshot, palm, held)
        proof['hand_rgb_rejection'] = str(exc)
        return proof
    proof['measurement'] = estimate_held(palm, snapshot, held, proof) if require_pose else None
    return proof
