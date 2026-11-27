import unittest,math
from source_view_candidates import near_front_candidates
class SourceViewTests(unittest.TestCase):
 def test_candidates_near_correct_furniture_not_origin_marker(self):
  t=dict(axis_aligned_bounds=dict(center=dict(x=1,y=-6.6),size=dict(x=1,y=1)),front_yaw=0,pose=dict(x=99,y=99))
  p=near_front_candidates(t,[[0,-8],[4,-8],[4,-3],[0,-3]])
  self.assertAlmostEqual(p[0][0],2.05);self.assertAlmostEqual(p[0][1],-6.6);self.assertAlmostEqual(abs(p[0][2]),math.pi)
 def test_room_filter_excludes_all_outside_candidates(self):
  t=dict(axis_aligned_bounds=dict(center=dict(x=3.9,y=0),size=dict(x=1,y=1)),front_yaw=0)
  self.assertEqual(near_front_candidates(t,[[3.4,-.5],[4.4,-.5],[4.4,.5],[3.4,.5]]),[])

 def test_side_views_available_when_front_has_no_room(self):
  t=dict(axis_aligned_bounds=dict(center=dict(x=3.9,y=0),size=dict(x=1,y=1)),front_yaw=0)
  points=near_front_candidates(t,[[0,-2],[4,-2],[4,2],[0,2]])
  self.assertTrue(points);self.assertTrue(all(p[0]<=4 for p in points))
