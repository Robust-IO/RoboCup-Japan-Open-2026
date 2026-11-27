import unittest
import numpy as np
from types import SimpleNamespace
from container_geometry import fit_release_xy,projection_inside
from container_live_adapter import ContainerLiveAdapter
class CenteringTests(unittest.TestCase):
 def opening(self):return dict(polygon_map_m=[[-.15,-.15],[.15,-.15],[.15,.15],[-.15,.15]],rim_height_m=.5)
 def test_edge_preference_no_longer_wins(self):
  offsets=np.array([[-.02,-.02],[.02,.02],[0.,0.]])
  q=fit_release_xy(self.opening(),offsets,[.10,0],margin=.03)
  np.testing.assert_allclose(q,[0,0],atol=1e-8)
 def test_offcenter_grip_centers_object_projection(self):
  offsets=np.array([[.01,-.02],[.09,.02],[0.,0.]])
  q=fit_release_xy(self.opening(),offsets,[.07,0],margin=.03)
  self.assertAlmostEqual(q[0],-.045)
  self.assertTrue(projection_inside(offsets+q,self.opening()['polygon_map_m'],margin=.03))
 def test_rotated_polygon_and_reverse_winding(self):
  o=self.opening();R=np.array([[.8,-.6],[.6,.8]])
  o['polygon_map_m']=(np.asarray(o['polygon_map_m'])@R.T).tolist()[::-1]
  q=fit_release_xy(o,np.array([[-.01,-.01],[.01,.01]]),[.1,0],margin=.03)
  np.testing.assert_allclose(q,[0,0],atol=1e-8)
 def test_inside_inset_does_not_stop_short_of_center(self):
  a=ContainerLiveAdapter.__new__(ContainerLiveAdapter);a.stages=SimpleNamespace(config={});a.opening=self.opening()
  a.plan=dict(placement_selection='maximum_rim_clearance',gripper_xy_map_m=[0,0])
  a.object_projection_map_m=np.array([[.06,-.02],[.10,.02]])
  self.assertFalse(a.release_geometry_ready([.08,0,.7],.6))
  a.object_projection_map_m=np.array([[-.02,-.02],[.02,.02]])
  self.assertTrue(a.release_geometry_ready([0,0,.7],.6))
