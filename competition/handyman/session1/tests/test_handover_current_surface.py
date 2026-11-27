import unittest
from types import SimpleNamespace
import numpy as np
from handover_current_surface import locate
from grasp_runtime.autograsp.tracking import UnobservedSurface
from grasp_runtime.autograsp.coordinator import AttemptFailure
class CurrentDeliveryTests(unittest.TestCase):
 def args(self):return (dict(frame='hand_palm_link',vertices_palm_m=[[0,0,0],[.02,.02,.02]],surface_rgb=[[1,2,3],[1,2,3]]),dict(translation=[0,0,0],quaternion=[0,0,0,1]),lambda q:np.eye(3),np.array([[.01,.01,.01]]),np.array([[1,2,3]]))
 def test_observed_shift_applied_to_delivery_center(self):
  fit=SimpleNamespace(translation=np.array([.03,0,0]),matched_points=[1]*40,residual_p90_m=.002,template_coverage=.8)
  c,d=locate(*self.args(),registration=lambda *a,**kw:fit);self.assertAlmostEqual(c[0],.04)
 def test_failure_does_not_accept_old_center(self):
  def fail(*a,**kw):raise UnobservedSurface('too_few_observed_surface_points')
  with self.assertRaises(AttemptFailure):locate(*self.args(),registration=fail)
