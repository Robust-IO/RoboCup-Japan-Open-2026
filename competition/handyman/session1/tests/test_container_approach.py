import unittest
import numpy as np
from candidate_selection import CandidateSelector
from container_approach import plan_approach
from room_entry import entered
class ApproachTests(unittest.TestCase):
 def setUp(self):
  self.poly=[[0,0],[6,0],[6,6],[0,6]]
  self.opening={'polygon_map_m':[[2.8,2.8],[3.2,2.8],[3.2,3.2],[2.8,3.2]]}
  grid=np.ones((120,120),bool);grid[56:64,56:64]=False
  self.sel=CandidateSelector(grid,.05,[0,0,0],.26,{'bin':{'axis_aligned_bounds':{'center':{'x':3,'y':3},'size':{'x':.4,'y':.4}}}})
 def test_arrives_from_each_side_without_exported_poses_or_yaw(self):
  for pose in ([1,3,2],[5,3,-2],[3,1,0],[3,5,1]):
   goal,region=plan_approach(self.sel,pose,self.opening,self.poly,'bin')
   self.assertLess(np.linalg.norm(np.array(goal[:2])-pose[:2]),1.3)
   self.assertTrue(entered([*goal[:2],goal[2]+2],region))
   self.assertFalse(entered([3,3,0],region))
 def test_current_region_does_not_require_other_goal(self):
  goal,region=plan_approach(self.sel,[2.2,3,2.5],self.opening,self.poly,'bin')
  self.assertTrue(entered([2.2,3,2.5],region))
 def test_obstructed_side_is_not_an_arrival_region(self):
  self.sel.walk[:,40:49]=False
  goal,region=plan_approach(self.sel,[5,3,0],self.opening,self.poly,'bin')
  self.assertFalse(entered([2.2,3,0],region))

 def test_early_nav_arrival_remains_inside_handoff_region(self):
  for pose in ([1,3,2],[5,3,-2],[3,1,0],[3,5,1]):
   goal,region=plan_approach(self.sel,pose,self.opening,self.poly,'bin')
   delta=np.array(pose[:2])-goal[:2];delta=delta/np.linalg.norm(delta)*.085
   self.assertTrue(entered([goal[0]+delta[0],goal[1]+delta[1],-2.8],region))

 def test_handoff_precedes_long_held_envelope_target_intersection(self):
  from container_approach import container_handoff_radius
  radius=container_handoff_radius(self.opening,self.sel.tables['bin'],.8866)
  self.assertGreater(radius,1.25)
  goal,region=plan_approach(self.sel,[1.75,3,0],self.opening,self.poly,'bin',handoff_radius_m=radius)
  self.assertTrue(entered([1.75,3,0],region))
  # It remains a traversable, visible region, not a blanket distance waiver.
  self.assertFalse(entered([3,3,0],region))
 def test_moderator_default_handoff_is_unchanged(self):
  _,region=plan_approach(self.sel,[1.75,3,0],self.opening,self.poly,'bin')
  self.assertFalse(entered([1.75,3,0],region))
