import unittest
import numpy as np
from grasp_runtime.autograsp.foreground_depth import foreground_mask
class ForegroundTests(unittest.TestCase):
 def test_distant_background_removed_without_rejecting_foreground(self):
  d=np.r_[np.linspace(1.48,1.58,2000),np.linspace(1.59,1.9,65),np.linspace(1.91,2,1200)]
  m=foreground_mask(d)
  self.assertTrue(m[:2000].all());self.assertFalse(m[-1200:].any())
 def test_continuous_object_is_not_split(self):
  d=np.linspace(1.4,1.8,1000)
  self.assertTrue(foreground_mask(d).all())
 def test_sparse_near_noise_does_not_hide_object(self):
  d=np.r_[1.,np.linspace(1.5,1.6,500)]
  self.assertTrue(foreground_mask(d)[1:].all())
