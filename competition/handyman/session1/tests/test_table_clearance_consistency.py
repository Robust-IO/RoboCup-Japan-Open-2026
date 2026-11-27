import unittest
from types import SimpleNamespace
import numpy as np
from grasp_runtime.autograsp.table_clearance import hand_table_clearance
from grasp_runtime.autograsp.planner import _minimum_palm_height,Policy
class ClearanceTests(unittest.TestCase):
 def test_planner_and_execution_use_same_per_part_margin(self):
  # A large offset belongs to the high part, not every low vertex.
  hand=SimpleNamespace(sweep=np.array([[[0,0,0]]*8,[[0,0,.1]]*8]),offset=np.array([.001,.04]),uncertainty=.002,environment_offset=.001)
  plane=np.array([.05,-.02,.4]);R=np.eye(3);xy=np.array([.2,.1]);p=Policy()
  z=_minimum_palm_height(xy,R,plane,hand,p)
  self.assertAlmostEqual(hand_table_clearance(np.r_[xy,z],R,plane,hand),p.table_margin_m)
  self.assertLess(hand_table_clearance(np.r_[xy,z-.001],R,plane,hand),p.table_margin_m)
