import unittest
from navigation_blockage import BlockageWatch
class BlockageTests(unittest.TestCase):
 def test_continuous_veto_requests_recovery(self):
  w=BlockageWatch()
  self.assertFalse(w.update(0,[0,0,0],True))
  self.assertFalse(w.update(3.9,[.001,0,0],True))
  self.assertTrue(w.update(4,[.001,0,0],True))
 def test_guest_clears_resets_wait(self):
  w=BlockageWatch();w.update(0,[0,0,0],True)
  self.assertFalse(w.update(3,[0,0,0],False))
  self.assertFalse(w.update(5,[0,0,0],True))
 def test_actual_progress_restarts_wait(self):
  w=BlockageWatch();w.update(0,[0,0,0],True)
  self.assertFalse(w.update(5,[.1,0,0],True))
