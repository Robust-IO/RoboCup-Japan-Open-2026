"""Pre-contact handoff: retain confirmed identity, measure NEW local RGBD geometry.

Apple-only (red surface / sphere); not an open-world detector or motion authority.
A stationary odom anchor bounds the query but NEVER supplies the measured center.
The anchor is local to one observer/task/epoch and is discarded after loss/contact.
"""
import copy
import json
import math
from pathlib import Path

import cv2
import numpy as np

from hand_camera_ray import rotation
from target_anchor_tracking import point_transform

SCHEMA = 'near_apple_geometry_v1'
CONTACT_XY = (.6892952185, .078)
PALM_HEIGHT_OFFSET = .2771978151685961


def check_gap(value):
    if type(value) not in (int, float) or not math.isfinite(value) or not .01 <= value <= .25:
        raise ValueError('invalid_near_geometry_gap')
    return value


def expected_base(anchor, snapshot):
    stamp = snapshot['streams']['depth']['stamp_ns']
    tf = snapshot['tracking_transforms']['odom_from_base_at_depth']
    # point_transform also validates the image-time frame binding.
    point_transform([0, 0, 0], tf, stamp=stamp, source='base_link', target='odom')
    return rotation(tf['quaternion']).T @ (np.asarray(anchor['point_odom_m']) - tf['translation'])


def prepared(snapshot):
    j = snapshot['joints']
    values = [j[k] for k in ('arm_lift_joint', 'arm_flex_joint', 'arm_roll_joint',
                             'wrist_flex_joint', 'wrist_roll_joint', 'hand_motor_joint')]
    return (np.isfinite(values).all() and .24 <= values[0] <= .30
            and np.all(np.abs(np.asarray(values[1:]) - [-1.7, 0, .1292036732051034, 0, .65]) <= .006))


def in_near_region(expected, snapshot, gap):
    return (prepared(snapshot) and -.020 <= expected[0] - CONTACT_XY[0] <= gap
            and abs(expected[1] - CONTACT_XY[1]) <= .06
            and abs(expected[2] - (snapshot['joints']['arm_lift_joint'] + PALM_HEIGHT_OFFSET)) <= .025)


def fit_current(rgb, depth, snapshot, anchor):
    """One projected local query; no detector box, classification or old depth."""
    expected = expected_base(anchor, snapshot)
    if depth.ndim != 2 or rgb.shape != (*depth.shape, 3):
        raise ValueError('near_geometry_shape_mismatch')
    k = np.asarray(snapshot['rgb_k'], float).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('invalid_intrinsics')
    tf = snapshot['base_from_rgb']; orient = rotation(tf['quaternion'])
    camera = orient.T @ (expected - np.asarray(tf['translation']))
    query_radius = anchor['radius_m'] + .025
    corners = camera + np.array([[x, y, z] for x in (-query_radius, query_radius)
        for y in (-query_radius, query_radius) for z in (-query_radius, query_radius)])
    if not np.isfinite(corners).all() or np.any(corners[:, 2] <= 0):
        raise ValueError('near_geometry_not_in_front_of_camera')
    pixels = corners[:, :2] / corners[:, 2:] * [k[0, 0], k[1, 1]] + k[:2, 2]
    h, w = depth.shape
    lo = np.maximum(0, np.floor(pixels.min(0)).astype(int) - 2)
    hi = np.minimum([w, h], np.ceil(pixels.max(0)).astype(int) + 3)
    x1, y1 = lo; x2, y2 = hi
    if x2 <= x1 or y2 <= y1:
        raise ValueError('near_geometry_outside_image')
    z = depth[y1:y2, x1:x2]
    v, u = np.indices(z.shape); u += x1; v += y1
    points = np.stack(((u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z), -1)
    points = points @ orient.T + tf['translation']
    red, green, blue = rgb[y1:y2, x1:x2].astype(float).transpose(2, 0, 1)
    mask = (np.isfinite(z) & (z > 0) & (z < 3) & (red > 80)
            & (red > 1.35*green) & (red > 1.2*blue) & (blue > .9*green)
            & (np.linalg.norm(points-expected, axis=2) <= query_radius))
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    components = np.flatnonzero(stats[1:, cv2.CC_STAT_AREA] >= 80) + 1
    if len(components) != 1:
        raise ValueError('unique_near_rgbd_surface_required')
    selected = labels == components[0]
    if (np.any(u[selected] <= 1) or np.any(u[selected] >= w-1)
            or np.any(v[selected] <= 1) or np.any(v[selected] >= h-1)):
        raise ValueError('near_geometry_clipped_at_image_edge')
    pts = points[selected]; origin = pts.mean(0); local = pts-origin
    sv = np.linalg.svd(local, compute_uv=False)
    if sv[0] <= 0 or sv[-1]/sv[0] < .035:
        raise ValueError('insufficient_near_surface_curvature')
    q = np.linalg.lstsq(np.c_[2*local, np.ones(len(local))], (local*local).sum(1), rcond=None)[0]
    radius2 = q[3] + q[:3] @ q[:3]
    if not .015**2 <= radius2 <= .065**2:
        raise ValueError('invalid_near_radius')
    center = q[:3]+origin; radius = float(np.sqrt(radius2))
    residual = float(np.quantile(abs(np.linalg.norm(pts-center, axis=1)-radius), .95))
    # Both checks refer to the immutable handoff anchor, not an accumulating prediction.
    if (not np.isfinite([*center, radius, residual]).all() or residual > .012
            or np.linalg.norm(center-expected) > .025 or abs(radius-anchor['radius_m']) > .008):
        raise ValueError('near_surface_no_longer_matches_locked_target')
    return dict(center_m=center.tolist(), radius_m=radius, fit_residual_p95_m=residual,
        points=len(pts), stamp_ns=snapshot['streams']['depth']['stamp_ns'],
        frame_id='base_link', source='rgbd', position_semantics='estimated_grasp_center',
        identity_confirmed=False, geometry_proposal_only=True, ready_for_motion=False,
        detector_confidence=None, local_query_roi=[int(x1), int(y1), int(x2), int(y2)],
        association_error_m=float(np.linalg.norm(center-expected)), motion_authorized=False)


def check_proof(proof, snapshot, *, task_id, epoch, measurement=None):
    if (proof.get('schema') != SCHEMA or proof.get('task_id') != task_id
            or proof.get('epoch') != epoch or snapshot.get('tracking_epoch') != epoch):
        raise ValueError('near_geometry_task_or_epoch_mismatch')
    check_gap(proof['gap_m']); anchor = proof['anchor']; seeds = anchor['identity_seeds']
    model=anchor.get('model_sha256')
    if not isinstance(model,str) or len(model)!=64 or any(c not in '0123456789abcdef' for c in model):
        raise ValueError('original_identity_model_binding_required')
    if (anchor.get('class_name') != 'apple' or len(seeds) != 3
            or not all(type(s['stamp_ns']) is int and s['stamp_ns'] > 0
                       and type(s['confidence']) in (float, int) and .8 <= s['confidence'] <= 1 for s in seeds)
            or not all(seeds[i]['stamp_ns'] < seeds[i+1]['stamp_ns'] for i in (0, 1))):
        raise ValueError('three_original_identity_seeds_required')
    point = np.asarray(anchor['point_odom_m'], float)
    if (point.shape != (3,) or not np.isfinite([*point, anchor['radius_m']]).all()
            or not .015 <= anchor['radius_m'] <= .065
            or type(anchor['stamp_ns']) is not int or anchor['stamp_ns'] < seeds[-1]['stamp_ns']
            or snapshot['streams']['depth']['stamp_ns'] <= anchor['stamp_ns']):
        raise ValueError('invalid_near_geometry_anchor')
    if not in_near_region(expected_base(anchor, snapshot), snapshot, proof['gap_m']):
        raise ValueError('near_geometry_not_in_precontact_region')
    if measurement is not None:
        stamp = snapshot['streams']['depth']['stamp_ns']
        tf = snapshot['tracking_transforms']['odom_from_base_at_depth']
        expected = anchor['point_odom_m']
        if (measurement.get('confidence') is not None
                or measurement.get('confidence_basis') != 'locked_identity_current_rgbd'
                or measurement.get('stamp_ns') != stamp
                or measurement.get('measurement') != 'fresh_rgbd'
                or not np.isfinite(measurement['point_odom_m']).all()
                or math.dist(measurement['point_odom_m'], expected) > .025
                or abs(measurement['radius_m']-anchor['radius_m']) > .008):
            raise ValueError('invalid_near_geometry_measurement')
        point_transform([0, 0, 0], tf, stamp=stamp, source='base_link', target='odom')


def verify_seed_files(proof, directory):
    """Recheck real saved classification seeds, not an invented confidence flag."""
    from apple_roi_confirmation import identity_evidence
    directory = Path(directory).resolve()
    for seed in proof['anchor']['identity_seeds']:
        capture = Path(seed['capture']).resolve()
        if capture.parent != directory:
            raise ValueError('identity_seed_outside_current_recording')
        obs = json.loads((capture/'observation.json').read_text())
        snap = json.loads((capture/'snapshot.json').read_text())
        if (obs.get('identity_tracking_task_id') != proof['task_id']
                or obs.get('tracking_epoch') != proof['epoch'] or snap.get('tracking_epoch') != proof['epoch']
                or obs.get('model_sha256') != proof['anchor']['model_sha256']
                or snap['streams']['depth']['stamp_ns'] != seed['stamp_ns']
                or obs.get('timing', {}).get('freshness_passed') is not True):
            raise ValueError('identity_seed_recording_binding_mismatch')
        measure = obs['identity_tracking']['measurement']
        if (obs['identity_tracking'].get('status') not in ('identity_seeding','identity_tracked')
                or measure.get('measurement')!='fresh_rgbd' or measure.get('class_name')!='apple'
                or measure.get('stamp_ns')!=seed['stamp_ns'] or measure.get('depth_valid') is not True
                or measure.get('clipped') is not False or measure.get('confidence')!=seed['confidence']):
            raise ValueError('original_classified_rgbd_seed_required')
        if measure.get('confidence_basis') == 'same_frame_near_grasp_roi':
            roi = identity_evidence(obs, snap)
            confidence = roi['confidence'] if roi else None
        elif measure.get('confidence_basis') == 'full_frame_detector':
            proposals = obs['tracking_proposals']
            confidence = proposals[0]['detection']['confidence'] if len(proposals) == 1 else None
            if len(proposals) == 1 and proposals[0]['detection']['name'] != 'apple':
                confidence = None
        else:
            confidence = None
        if confidence != seed['confidence'] or confidence is None or confidence < .8:
            raise ValueError('original_identity_seed_not_confirmed')


def recorded_candidate(observation, snapshot, rgb, depth, *, directory=None):
    """Recompute current geometry for closure/lift audits; saved centers aren't proof."""
    evidence = observation['identity_tracking']; proof = evidence['near_geometry_proof']
    check_proof(proof, snapshot, task_id=observation['identity_tracking_task_id'],
                epoch=observation['tracking_epoch'], measurement=evidence['measurement'])
    if directory is not None:
        verify_seed_files(proof, directory)
    value = fit_current(rgb, depth, snapshot, proof['anchor'])
    measured = evidence['measurement']
    point = point_transform(value['center_m'], snapshot['tracking_transforms']['odom_from_base_at_depth'],
                            stamp=value['stamp_ns'], source='base_link', target='odom')
    if math.dist(point, measured['point_odom_m']) > 1e-5 or abs(value['radius_m']-measured['radius_m']) > 1e-5:
        raise ValueError('saved_near_geometry_differs_from_current_pixels')
    return value


class NearAppleGeometry:
    def __init__(self, task_id, epoch, gap_m=.15):
        self.task_id, self.epoch, self.gap_m = task_id, epoch, check_gap(gap_m)
        self.reset()

    def reset(self):
        self.seeds = []; self.anchor = None; self.active = False; self.last_stamp = None

    def remember_identity(self, evidence, result, snapshot, capture):
        if evidence.get('status') == 'reobserve':
            self.reset(); return
        item = evidence.get('measurement')
        if not item or capture is None:
            return
        if not evidence['identity_confirmed'] or len(self.seeds) < 3:
            if item['confidence'] < .8:
                self.reset(); return
            self.seeds.append(dict(stamp_ns=item['stamp_ns'], confidence=item['confidence'], capture=str(capture)))
            self.seeds = self.seeds[-3:]
        if evidence['identity_confirmed'] and len(self.seeds) == 3:
            self.anchor = dict(class_name='apple', point_odom_m=list(item['point_odom_m']),
                radius_m=item['radius_m'], stamp_ns=item['stamp_ns'], identity_seeds=copy.deepcopy(self.seeds),
                model_sha256=result.get('model_sha256'))

    def ready(self, snapshot):
        if self.active:
            return True  # An invalid next frame must invalidate the handoff, not bypass it.
        if self.anchor is None:
            return False
        try:
            return (snapshot.get('tracking_epoch') == self.epoch
                    and in_near_region(expected_base(self.anchor, snapshot), snapshot, self.gap_m))
        except (KeyError, ValueError, TypeError):
            return False

    def measure(self, rgb, depth, snapshot, now_ns):
        stamp = snapshot['streams']['depth']['stamp_ns']; rgb_stamp = snapshot['streams']['rgb']['stamp_ns']
        if (not all(type(t) is int and t > 0 and 0 <= (now_ns-t)/1e9 <= .4 for t in (stamp, rgb_stamp))
                or abs(stamp-rgb_stamp) > 30_000_000):
            raise ValueError('fresh_synchronised_near_rgbd_required')
        if not -.02 <= (now_ns-snapshot['joint_stamp_ns'])/1e9 <= .4:
            raise ValueError('fresh_near_joint_snapshot_required')
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError('duplicate_or_reversed_near_geometry')
        proof = dict(schema=SCHEMA, task_id=self.task_id, epoch=self.epoch,
                     gap_m=self.gap_m, anchor=copy.deepcopy(self.anchor))
        check_proof(proof, snapshot, task_id=self.task_id, epoch=self.epoch)
        candidate = fit_current(rgb, depth, snapshot, self.anchor)
        point = point_transform(candidate['center_m'], snapshot['tracking_transforms']['odom_from_base_at_depth'],
                                stamp=stamp, source='base_link', target='odom')
        measurement = dict(point_odom_m=point, stamp_ns=stamp, confidence=None,
            full_frame_confidence=None, confidence_basis='locked_identity_current_rgbd',
            radius_m=candidate['radius_m'], fit_residual_p95_m=candidate['fit_residual_p95_m'],
            measurement='fresh_rgbd', class_name='apple', depth_valid=True, clipped=False)
        self.active = True; self.last_stamp = stamp
        return dict(status='near_geometry_tracked', identity_confirmed=True,
            identity_basis='previous_three_classified_frames_then_current_local_rgbd_no_reclassification',
            identity_reclassified=False, measurement=measurement, near_geometry_proof=proof,
            current_geometry_candidate=candidate, motion_authorized=False, grasp_center_verified=False)
