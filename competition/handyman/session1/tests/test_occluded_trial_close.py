import unittest
from grasp_runtime.autograsp.occluded_trial_close import decide

class TrialCloseTests(unittest.TestCase):
 def args(self):return [dict(fits=False,reason='contact_band_unobserved'),dict(fits=True,clearance_m=.1),dict(depth_measured=True,stamp_ns=10),10,[-.0004301875,.0008432227,-.0001038386]]
 def test_recorded_alignment_permits_trial_not_success(self):
  r=decide(*self.args());self.assertTrue(r['fits']);self.assertTrue(r['trial_close']);self.assertFalse(r['grasp_verified']);self.assertFalse(r['current_contact_band_observed'])
 def test_stale_or_propagated_position_rejected(self):
  for patch in [dict(stamp_ns=9),dict(depth_measured=False)]:
   a=self.args();a[2].update(patch);self.assertFalse(decide(*a)['fits'])
 def test_misalignment_is_not_waived(self):
  a=self.args();a[4]=[.03,0,0];self.assertFalse(decide(*a)['fits'])
 def test_observed_collision_or_width_failure_is_not_waived(self):
  for reason in ['actual_closing_hand_too_low','material_outside_actual_open_gap','neck_not_enclosed_in_pad_depth']:
   a=self.args();a[0]['reason']=reason;self.assertFalse(decide(*a)['fits'])
 def test_reference_must_fit_finger_region(self):
  a=self.args();a[1]=dict(fits=False,reason='neck_not_enclosed_in_pad_depth');self.assertFalse(decide(*a)['fits'])
