import unittest
from threading import Event
from types import SimpleNamespace
from unittest.mock import patch
from edge_view import recenter_head
from furniture_reobservation import observe_furniture
class EdgeTests(unittest.TestCase):
 def test_recorded_ketchup_top_edge_raises_head(self):
  g=recenter_head([{'confidence':.386,'box':[211,0,235,20]}],(480,640),(0,-.5236),[554,0,320,0,554,240,0,0,1])
  self.assertGreater(g[1],-.5236);self.assertLessEqual(g[1],0)
 def test_centered_target_does_not_request_move(self):
  self.assertIsNone(recenter_head([{'confidence':.9,'box':[300,200,340,280]}],(480,640),(0,0),[554,0,320,0,554,240,0,0,1]))
 def test_failed_observation_retained_until_next_view_succeeds(self):
  from grasp_runtime.autograsp.coordinator import AttemptFailure
  from grasp_runtime.autograsp.recovery import Failure
  s=SimpleNamespace(state=lambda c:{'poses':{'map':[0,0,0]}},check=lambda c:None,candidate_selector=object(),grasp_backend=SimpleNamespace(record=lambda r:None))
  with patch('furniture_reobservation._observe_cycle',side_effect=[AttemptFailure(Failure.OCCLUDED),{'confirmed':True}]):
   self.assertTrue(observe_furniture(s,{},Event())['confirmed'])
 def test_cancelled_recovery_exits(self):
  e=Event();s=SimpleNamespace(state=lambda c:{'poses':{'map':[0,0,0]}},check=lambda c:None,candidate_selector=object(),grasp_backend=SimpleNamespace(record=lambda r:e.set()))
  def check(c):
   if c.is_set():raise RuntimeError('task_cancelled')
  s.check=check
  with patch('furniture_reobservation._observe_cycle',return_value={'confirmed':False}):
   with self.assertRaisesRegex(RuntimeError,'task_cancelled'):observe_furniture(s,{},e)
