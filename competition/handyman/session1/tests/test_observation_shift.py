import math,unittest
import numpy as np
from types import SimpleNamespace
from threading import Event
from furniture_reobservation import shift_observation_view
class ShiftTests(unittest.TestCase):
 def stages(self):
  state={'poses':{'map':[0,0,math.pi/2]},'joints':[0,0,0,0,0,.65]};calls=[]
  def adjust(label,start,delta,*a):
   calls.append(delta);state['poses']['map']=[-.2,0,start[2]]
  selector=SimpleNamespace(walk=np.ones((100,100),bool),cell=lambda q:(int((q[1]+1)/.02),int((q[0]+1)/.02)))
  s=SimpleNamespace(state=lambda c:state,check=lambda c:None,candidate_selector=selector,motion=SimpleNamespace(adjust=adjust),grasp_backend=SimpleNamespace(record=lambda r:None))
  return s,calls
 def test_shift_does_not_turn_towards_travel_direction(self):
  s,c=self.stages();self.assertTrue(shift_observation_view(s,[-.2,0,-1],Event()))
  self.assertAlmostEqual(c[0][0],0);self.assertAlmostEqual(c[0][1],.2);self.assertEqual(c[0][2],0)
 def test_blocked_straight_segment_skipped_without_commands(self):
  s,c=self.stages();s.candidate_selector.walk[50,45]=False
  self.assertFalse(shift_observation_view(s,[-.2,0,-1],Event()));self.assertFalse(c)
 def test_distant_view_not_sent_as_unbounded_local_shift(self):
  s,c=self.stages();self.assertFalse(shift_observation_view(s,[-2,0,0],Event()));self.assertFalse(c)
