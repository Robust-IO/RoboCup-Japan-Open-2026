import unittest,math
from candidate_selection import destination_candidates
class ShiftTests(unittest.TestCase):
 def test_only_configured_destination_moves(self):
  p=[[1.,2.,math.pi/2]]
  self.assertEqual(destination_candidates(p,{}),p)
  q=destination_candidates(p,{'session_profile':{'destination':{'candidate_forward_shift_m':.2}}})
  self.assertAlmostEqual(q[0][0],1);self.assertAlmostEqual(q[0][1],2.2)
  self.assertEqual(q[0][2],p[0][2]);self.assertEqual(p[0][1],2)

 def test_container_sides_face_opening_and_preserve_distance(self):
  config={'destination_id':'bin','session_profile':{'destination':{'mode':'container_drop'}},'container_opening':{'destination_id':'bin','polygon_map_m':[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]}}
  q=destination_candidates([[0,-1,math.pi/2]],config)
  self.assertEqual(len(q),4)
  for x,y,yaw in q:
   self.assertAlmostEqual(math.hypot(x,y),1)
   self.assertAlmostEqual(math.cos(yaw),-x)
   self.assertAlmostEqual(math.sin(yaw),-y)
  from candidate_selection import CandidateSelector
  import numpy as np
  selector=CandidateSelector(np.ones((60,60),bool),.1,[-3,-3,0],.1,{'bin':{'axis_aligned_bounds':{'center':{'x':0,'y':0},'size':{'x':.4,'y':.4}}}})
  ranked,_=selector.rank([1.5,0,0],q,'bin')
  self.assertAlmostEqual(ranked[0][0],1)
  self.assertAlmostEqual(ranked[0][1],0)
  region=[[-2,-2],[.5,-2],[.5,2],[-2,2]]
  self.assertEqual(len(destination_candidates([[0,-1,math.pi/2]],config,region)),3)
