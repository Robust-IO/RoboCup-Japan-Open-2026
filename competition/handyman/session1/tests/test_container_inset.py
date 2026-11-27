import unittest
import numpy as np
from types import SimpleNamespace
from container_geometry import fit_release_xy,projection_inside
from container_live_adapter import ContainerLiveAdapter
class InsetTests(unittest.TestCase):
 def opening(self):return dict(polygon_map_m=[[-.2,-.15],[.2,-.15],[.2,.15],[-.2,.15]],rim_height_m=.5)
 def test_plan_moves_edge_preference_inward_and_preserves_map(self):
  o=self.opening();original=[p[:] for p in o['polygon_map_m']]
  offsets=np.array([[-.02,-.02],[.02,.02],[0.,0.]])
  xy=fit_release_xy(o,offsets,[.18,0],margin=.03)
  self.assertLessEqual(xy[0],.1500001)
  self.assertTrue(projection_inside(offsets+xy,o['polygon_map_m'],margin=.03))
  self.assertEqual(o['polygon_map_m'],original)
 def test_small_opening_is_not_silently_widened(self):
  o=dict(polygon_map_m=[[-.025,-.025],[.025,-.025],[.025,.025],[-.025,.025]])
  with self.assertRaises(ValueError):fit_release_xy(o,np.array([[0.,0.]]),[0.,0.],margin=.03)
 def test_object_projection_ignored_but_gripper_inset_enforced(self):
  a=ContainerLiveAdapter.__new__(ContainerLiveAdapter);a.opening=self.opening();a.stages=SimpleNamespace(config={})
  a.object_projection_map_m=np.array([[.16,0],[.18,.01]])
  self.assertTrue(a.release_geometry_ready([0,0,.7],.6))
  self.assertFalse(a.release_geometry_ready([.18,0,.7],.6))
  a.object_projection_map_m=np.array([[-.02,0],[.02,.01]])
  self.assertTrue(a.release_geometry_ready([0,0,.7],.6))
