import unittest
from grasp_runtime.autograsp.grasp_confirmation import grip_retained, lift_sample_passes


class GraspConfirmationTests(unittest.TestCase):
    def test_opening_alone_does_not_prove_grasp(self):
        self.assertTrue(grip_retained(.4, .4))
        self.assertFalse(lift_sample_passes([0,0,0], [0,0,.08], True, 2, 1))

    def test_following_object_with_retained_grip_passes(self):
        self.assertTrue(lift_sample_passes([.002,0,.078], [0,0,.08], True, 2, 1))

    def test_slip_or_opening_change_fails(self):
        self.assertFalse(grip_retained(.5, .4))
        self.assertFalse(lift_sample_passes([0,0,.05], [0,0,.09], True, 2, 1))
        self.assertFalse(lift_sample_passes([0,0,.08], [0,0,.08], False, 2, 1))

    def test_duplicate_frame_and_nonfinite_feedback_fail(self):
        self.assertFalse(lift_sample_passes([0,0,.08], [0,0,.08], True, 1, 1))
        self.assertFalse(lift_sample_passes([0,0,float('nan')], [0,0,.08], True, 2, 1))
        self.assertFalse(grip_retained(float('nan'), .4))
