import unittest
from grasp_runtime.autograsp.identity_policy import source_association
class SofaAssociationTests(unittest.TestCase):
 def test_recorded_sofa_views(self):
  t={'axis_aligned_bounds':{'center':{'x':.8488384485,'y':-6.72358799},'size':{'x':.8328487873,'y':.8399963379}}}
  for p in [[1.3406,-6.6679,.5445],[1.3751,-6.7032,.5468],[1.3332,-6.6101,.5413]]:
   self.assertTrue(source_association(p,t,.985))
   self.assertFalse(source_association(p,t,.8))
  self.assertFalse(source_association([1.5,-6.7,.5],t,.999))
  self.assertFalse(source_association([float('nan'),-6.7,.5],t,.999))
