import unittest
from carry_grip_watch import GripWatch
class GripWatchTests(unittest.TestCase):
 def test_held_feedback_never_periodically_stops(self):
  w=GripWatch(.37)
  for i in range(6000):self.assertFalse(w.observe(.37,i*100_000_000,.1))
 def test_three_fresh_empty_samples_trigger_confirmation(self):
  w=GripWatch(.37)
  self.assertFalse(w.observe(0,1,.1))
  self.assertFalse(w.observe(0,150000001,.1))
  self.assertTrue(w.observe(0,300000001,.1))
 def test_duplicate_stale_and_single_outlier_do_not_trigger(self):
  w=GripWatch(.37)
  for i in range(10):self.assertFalse(w.observe(0,1,.1))
  self.assertFalse(w.observe(0,300000001,2.))
  self.assertFalse(w.observe(.37,400000001,.1))
  self.assertFalse(w.observe(0,500000001,.1))
