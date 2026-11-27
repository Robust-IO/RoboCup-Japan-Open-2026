import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from penguin_geometry import estimate_source
from penguin_hand_hold import (create_reference, verify_hand, contact_context,
                               verify_lift, estimate_held, palm_at_capture, hold_evidence)
from penguin_place import verify_placed
from test_penguin_geometry import rendered_penguin


class PenguinHandHoldTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        rgb, depth, self.before, detection, _, _ = rendered_penguin()
        self.model = estimate_source(rgb, depth, self.before, detection)
        self.measurement = dict(self.model, current_head_depth_measured=True)
        head = np.asarray(self.model['cap_center_m'])
        hand = head - [0, 0, .1]
        identity = dict(translation=[0, 0, 0], quaternion=[0, 0, 0, 1], stamp_ns=1_000_000_000)
        self.before.update(tracking_epoch='live-one', hand_k=[205, 0, 320, 0, 205, 240, 0, 0, 1],
                           hand_d=[0]*5, palm_base_at_depth=hand.tolist(),
                           base_from_hand=dict(translation=hand.tolist(), quaternion=[0, 0, 0, 1]),
                           palm_from_hand=dict(translation=[0, 0, 0], quaternion=[0, 0, 0, 1]),
                           joints=dict(hand_motor_joint=.65, arm_lift_joint=.25),
                           tracking_transforms=dict(
                               odom_from_base_at_depth=dict(identity, source_frame='base_link', target_frame='odom'),
                               base_from_odom_at_hand=dict(identity, source_frame='odom', target_frame='base_link')))
        self.before['streams']['hand'] = dict(width=640, height=480, stamp_ns=1_000_000_000, frame='hand_camera_rgb_frame')
        self.image = np.full((480, 640, 3), 170, np.uint8)
        cv2.rectangle(self.image, (225, 160), (250, 330), (15, 15, 15), -1)
        cv2.rectangle(self.image, (390, 160), (415, 330), (15, 15, 15), -1)
        cv2.circle(self.image, (320, 240), 60, (60, 60, 60), -1)
        cv2.circle(self.image, (330, 215), 10, (245, 245, 245), 4)
        cv2.circle(self.image, (330, 265), 10, (245, 245, 245), 4)
        cv2.circle(self.image, (280, 240), 18, (20, 180, 200), -1)
        self.reference = create_reference(self.image, self.before, self.measurement,
                                          Path(self.tmp.name)/'face.png', model=self.model)
        self.after = copy.deepcopy(self.before)
        for name in ('rgb', 'depth', 'hand'):
            self.after['streams'][name]['stamp_ns'] = 2_000_000_000
        for tf in self.after['tracking_transforms'].values():
            tf['stamp_ns'] = 2_000_000_000
        self.after['joints']['hand_motor_joint'] = .3
        self.after['joints']['arm_lift_joint'] += .1
        self.after['palm_base_at_depth'][2] += .1
        self.after['base_from_hand']['translation'][2] += .1

    def evidence(self, image=None, snapshot=None):
        return verify_hand(self.image if image is None else image,
                           self.after if snapshot is None else snapshot, self.reference, .3,
                           now_ns=2_050_000_000)

    def test_calibrated_reference_and_current_face_are_image_evidence_only(self):
        evidence = self.evidence()
        self.assertGreater(evidence['correlation'], .99)
        self.assertFalse(evidence['measured_3d_object_position_available'])
        self.assertFalse(evidence['new_target_identity_created'])
        self.assertEqual(self.reference['raster_profile'], 'sigverse_hsr_hand_v2')
        self.assertFalse(self.reference['motion_authorized'])

    def test_small_target_shift_allowed_but_missing_colored_target_rejected(self):
        shifted = cv2.warpAffine(self.image, np.float32([[1, 0, 4], [0, 1, -3]]), (640, 480))
        self.assertEqual(self.evidence(shifted)['pixel_shift'], [4, -3])
        missing = self.image.copy()
        cv2.circle(missing, (280, 240), 24, (60, 60, 60), -1)
        with self.assertRaises(ValueError):
            self.evidence(missing)

    def test_old_epoch_old_image_or_lost_contact_cannot_verify_hold(self):
        for key, value in (('tracking_epoch', 'another-run'),):
            changed = copy.deepcopy(self.after);changed[key] = value
            with self.assertRaises(ValueError): self.evidence(snapshot=changed)
        changed = copy.deepcopy(self.after);changed['joints']['hand_motor_joint'] = 0.
        with self.assertRaises(ValueError): self.evidence(snapshot=changed)
        with self.assertRaisesRegex(ValueError, 'fresh_penguin_hand_image'):
            verify_hand(self.image, self.after, self.reference, .3, now_ns=2_600_000_000)

    def test_lift_needs_measured_palm_and_joint_motion_as_well_as_face(self):
        proof = verify_lift(self.before, self.after, self.evidence(), .3)
        self.assertTrue(proof['grasp_lift_verified'])
        self.assertFalse(proof['object_depth_independently_measured'])
        stationary = copy.deepcopy(self.after)
        stationary['palm_base_at_depth'] = self.before['palm_base_at_depth']
        with self.assertRaises(ValueError): verify_lift(self.before, stationary, self.evidence(), .3)

    def test_held_xyz_is_explicit_estimate_and_cannot_prove_final_placement(self):
        contact = contact_context(self.model, self.measurement, self.before, self.reference, .3)
        self.assertFalse(contact['grasp_lift_verified'])
        palm = palm_at_capture(self.after)
        with self.assertRaises(ValueError): estimate_held(palm, self.after, contact, self.evidence())
        held = dict(contact, grasp_lift_verified=True)
        estimate = estimate_held(palm, self.after, held, self.evidence())
        np.testing.assert_allclose(estimate['center_m'], np.asarray(self.measurement['center_m'])+[0, 0, .1], atol=1e-9)
        self.assertTrue(estimate['predicted_center_returned'])
        self.assertFalse(estimate['current_head_depth_measured'])
        self.assertEqual(estimate['original_object_depth_stamp_ns'], 1_000_000_000)
        with self.assertRaisesRegex(ValueError, 'independent_penguin'):
            verify_placed([dict(measurement=estimate)]*3, {}, [0, 0, 0, 0, 0, .65], cleanup_completed=True)

    def test_reference_requires_measured_head_and_consistent_capture_transforms(self):
        bad = dict(self.measurement, current_head_depth_measured=False)
        with self.assertRaises(ValueError):
            create_reference(self.image, self.before, bad, Path(self.tmp.name)/'bad.png', model=self.model)
        bad = copy.deepcopy(self.before);bad['palm_base_at_depth'][0] += .02
        with self.assertRaises(ValueError): palm_at_capture(bad)

    def test_failed_rgb_can_use_actual_depth_but_cannot_create_kinematic_pose(self):
        contact = contact_context(self.model, self.measurement, self.before, self.reference, .3)
        held = dict(contact, grasp_lift_verified=True)
        missing = self.image.copy();cv2.circle(missing, (280, 240), 24, (60, 60, 60), -1)
        sample = dict(hand=missing, rgb=None, depth=None, snapshot=self.after)
        actual = dict(current_depth_surface_verified=True, measurement=dict(position_source='current_depth'))
        with patch('penguin_hand_hold.current_depth', return_value=actual) as depth, \
                patch('penguin_hand_hold.estimate_held') as estimate:
            proof = hold_evidence(sample, palm_at_capture(self.after), held, require_pose=True)
        depth.assert_called_once();estimate.assert_not_called()
        self.assertTrue(proof['current_depth_surface_verified'])
        with patch('penguin_hand_hold.current_depth', side_effect=ValueError('no_current_depth')):
            with self.assertRaises(ValueError):hold_evidence(sample, palm_at_capture(self.after), held)
        wrong = dict(sample, snapshot=copy.deepcopy(self.after));wrong['snapshot']['tracking_epoch']='old-context'
        with patch('penguin_hand_hold.current_depth') as depth:
            with self.assertRaises(ValueError):hold_evidence(wrong, palm_at_capture(self.after), held)
        depth.assert_not_called()


if __name__ == '__main__':
    unittest.main()
