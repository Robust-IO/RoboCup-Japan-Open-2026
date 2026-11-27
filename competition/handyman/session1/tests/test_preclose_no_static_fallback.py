import unittest
from test_preclose_alignment import PrecloseTests
from grasp_runtime.autograsp.preclose_alignment import align_before_close
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure
class NoStaticTests(unittest.TestCase):
 def test_fallback_flags_do_not_authorize_closure(self):
  for flag in ('static_anchor_authorized','occlusion_anchor_authorized'):
   b,c=PrecloseTests().backend(iter([]))
   b._observe_tracking=lambda:({'snapshot':{}},{'depth_measured':False,flag:True})
   with self.assertRaises(AttemptFailure):align_before_close(b)
   self.assertFalse(c)
 def test_registration_failure_restores_state_and_does_not_close(self):
  b,c=PrecloseTests().backend(iter([]));b.static_precontact_anchor=True
  def fail():
   self.assertFalse(b.static_precontact_anchor);self.assertFalse(b.pose_locked)
   raise AttemptFailure(Failure.OCCLUDED,'surface_moved_beyond_registration_window')
  b._observe_tracking=fail
  with self.assertRaises(AttemptFailure):align_before_close(b)
  self.assertTrue(b.pose_locked);self.assertTrue(b.static_precontact_anchor);self.assertFalse(c)
