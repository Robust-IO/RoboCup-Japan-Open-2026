import unittest
from grasp_runtime.autograsp.occluded_trial_close import session6_depth_override
from grasp_runtime.autograsp.planner import Policy
class DepthOverrideTests(unittest.TestCase):
 def args(self):return [dict(fits=False,reason='contact_band_unobserved'),dict(depth_measured=True,stamp_ns=10),10,[0.,0.,0.],dict(observed_width_m=.035),Policy(),.02]
 def test_missing_band_allows_attempt_not_success(self):
  r=session6_depth_override(*self.args());self.assertTrue(r['fits']);self.assertFalse(r['grasp_verified'])
 def test_other_failures_unchanged(self):
  a=self.args();a[0]['reason']='neck_not_enclosed_in_pad_depth';self.assertFalse(session6_depth_override(*a)['fits'])
 def test_misaligned_wide_low_stale_rejected(self):
  for index,value in [(3,[.03,0,0]),(4,dict(observed_width_m=.2)),(6,-.01),(1,dict(depth_measured=True,stamp_ns=9))]:
   a=self.args();a[index]=value;self.assertFalse(session6_depth_override(*a)['fits'])
