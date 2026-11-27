import unittest
import numpy as np
from candidate_selection import CandidateSelector
class CandidateSelectionTests(unittest.TestCase):
 def selector(self,grid):return CandidateSelector(grid,1,[0,0,0],0,{'table':{'axis_aligned_bounds':{'center':{'x':8,'y':5},'size':{'x':2,'y':2}}}})
 def test_blocked_view_rejected_even_if_nearer(self):
  grid=np.ones((12,12),bool);grid[5,5]=False
  ranked,audit=self.selector(grid).rank([2.5,5.5,0],[[3.5,5.5,0],[7.5,8.5,0]],'table')
  self.assertEqual(ranked,[[7.5,8.5,0]])
  self.assertEqual(audit['candidates'][0]['rejected_reason'],'view_blocked_on_static_map')
 def test_distance_uses_detour_and_does_not_cross_wall(self):
  grid=np.ones((10,10),bool);grid[:8,4]=False
  s=self.selector(grid);d=s.lengths((2,2),[(2,6),(6,2)])
  self.assertGreater(d[(2,6)],d[(6,2)])
 def test_unknown_or_occupied_disconnects_destination(self):
  grid=np.ones((10,10),bool);grid[:,4]=False
  self.assertNotIn((2,6),self.selector(grid).lengths((2,2),[(2,6)]))
 def test_target_table_does_not_block_its_own_view(self):
  grid=np.ones((12,12),bool);grid[4:7,7:10]=False
  self.assertTrue(self.selector(grid).clear_view([4.5,5.,0],self.selector(grid).tables['table']))
