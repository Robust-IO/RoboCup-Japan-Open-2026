import unittest
from unittest.mock import patch
import test_carry_clearance
class TargetedEscapeTests(unittest.TestCase):
 def test_partial_turn_does_not_require_full_circle(self):
  g=test_carry_clearance.CarryTests().scene()
  with patch.object(g,'turn_clear',return_value=False):
   plan=g.escape([.8,1.5,0.],target_yaw=.3)
   self.assertEqual(plan['distance_m'],0.)
   self.assertTrue(g.choose_turn(plan['pose'],.3)['clear'])
 def test_translate_to_clear_actual_turn(self):
  g=test_carry_clearance.CarryTests().scene()
  with patch.object(g,'turn_clear',return_value=False):
   plan=g.escape([1.4,1.5,0.],target_yaw=1.57)
   self.assertGreater(plan['distance_m'],0.)
   self.assertEqual(plan['delta'][2],0.)
   self.assertTrue(g.choose_turn(plan['pose'],1.57)['clear'])
 def test_blocked_map_is_not_bypassed(self):
  g=test_carry_clearance.CarryTests().scene();g.selector.free[:]=False;g.clearance[:]=0.
  with self.assertRaises(ValueError):g.escape([1.4,1.5,0.],target_yaw=.3)
