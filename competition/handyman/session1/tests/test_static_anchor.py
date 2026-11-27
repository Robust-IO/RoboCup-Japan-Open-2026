import unittest
from types import SimpleNamespace
from grasp_runtime.autograsp.static_anchor import allowed,retain
class StaticAnchorTests(unittest.TestCase):
 def backend(self):return SimpleNamespace(static_precontact_anchor=True,preextension_anchor_confirmed=True,holding_state=lambda:'empty')
 def test_registration_shift_before_contact_retains(self):self.assertTrue(allowed(self.backend(),'surface_moved_beyond_registration_window'))
 def test_no_previous_attempt_reuse(self):
  b=self.backend();b.preextension_anchor_confirmed=False
  self.assertFalse(allowed(b,'surface_moved_beyond_registration_window'))
 def test_after_contact_and_lift_require_observation(self):
  b=self.backend();b.static_precontact_anchor=False
  self.assertFalse(allowed(b,'insufficient_geometry_colour_correspondences'))
  self.assertFalse(allowed(self.backend(),'surface_moved_beyond_registration_window',[0,0,.03]))
 def test_unrelated_errors_not_bypassed(self):self.assertFalse(allowed(self.backend(),'invalid_registration_arrays'))
 def test_not_claimed_as_measured(self):
  b=self.backend();b._observe_locked=lambda:({},dict(center_m=[1,2,3]));b.record=lambda e:None
  _,m=retain(b,'surface_moved_beyond_registration_window')
  self.assertFalse(m['depth_measured']);self.assertTrue(m['static_anchor_authorized'])
