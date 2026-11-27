import math,unittest
import numpy as np
from table_approach_reachability import base_path,hand_path
TABLE=dict(axis_aligned_bounds=dict(center=dict(x=1.971,y=4.024),size=dict(x=.72,y=1.2)))
class ReachTest(unittest.TestCase):
 def test_previous_109cm_command_rejected(self):
  self.assertFalse(base_path([2.0976,2.5751,1.6497],[1.09138,.03385,0],TABLE)['ready'])
 def test_near_edge_full_approach_accepted(self):
  self.assertTrue(base_path([2.0976,2.5751,math.pi/2],[.39,0,0],TABLE)['ready'])
 def test_clear_long_move_has_no_fixed_cap(self):
  self.assertTrue(base_path([0,0,0],[1.2,0,0],TABLE)['ready'])
 def test_endpoints_outside_but_segment_crosses_table_rejected(self):
  self.assertFalse(base_path([1.97,2.5,math.pi/2],[3.,0,0],TABLE)['ready'])
 def test_clear_patch_does_not_hide_intervening_obstacle(self):
  p=np.array([[.5,0,.50],[1.,.3,.44]])
  self.assertFalse(hand_path(p,[.3,0],[.8,0],[0,0,.44],.48,.05)['ready'])
  self.assertTrue(hand_path(p,[.3,.3],[.8,.3],[0,0,.44],.48,.05)['ready'])
if __name__=='__main__':unittest.main()
