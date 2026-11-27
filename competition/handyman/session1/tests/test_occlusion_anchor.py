import unittest
from grasp_runtime.autograsp.occlusion_anchor import allowed
class OcclusionAnchorTests(unittest.TestCase):
 def test_same_attempt_geometrically_occluded_may_rotate(self):
  self.assertTrue(allowed('insufficient_geometry_colour_correspondences',True,.7))
 def test_measured_motion_never_uses_old_anchor(self):
  self.assertFalse(allowed('surface_moved_beyond_registration_window',True,1.))
 def test_unexplained_failure_not_assumed_occlusion(self):
  self.assertFalse(allowed('insufficient_geometry_colour_correspondences',True,.1))
 def test_previous_attempt_anchor_not_reused(self):
  self.assertFalse(allowed('only_tiny_surface_fragment_visible',False,.9))
