import unittest
import numpy as np
from measured_release_posture import MeasuredReleaseCore
from support_reference import assess_bottom
from grasp_runtime.autograsp.kinematics import forward
class SideReleaseTests(unittest.TestCase):
 def test_rolled_grasp_support_restored(self):
  joints=[.4,-1.7,0,.1292036732051034,np.pi/6,.25]
  R=forward(joints)[1];normal=R.T@np.array([0,0,1.])
  c=dict(vertices_palm_m=[[0,0,0],[.02,.02,.02]],source_support_reference=dict(normal_palm=normal.tolist(),point_palm_m=[0,0,0],basis='measured_pre_lift_support_plane'))
  neutral=list(joints);neutral[4]=0
  self.assertFalse(assess_bottom(c,forward(neutral)[1])['established'])
  self.assertTrue(assess_bottom(c,MeasuredReleaseCore(None,joints).rotation)['established'])
