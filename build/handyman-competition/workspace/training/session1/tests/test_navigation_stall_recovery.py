import unittest
from navigation_stall_recovery import door_pose_ready,needs_escape
class StallTests(unittest.TestCase):
 def test_near_door_with_correct_heading(self):self.assertTrue(door_pose_ready(dict(kind='door_entry',pose=[0,0,0]),[.063,0,.035],[.085,.08]))
 def test_door_wrong_heading_not_skipped(self):self.assertFalse(door_pose_ready(dict(kind='door_entry',pose=[0,0,0]),[.063,0,.2],[.085,.08]))
 def test_furniture_goal_unchanged(self):self.assertFalse(door_pose_ready(dict(kind='final',pose=[0,0,0]),[.063,0,0],[.085,.08]))
 def test_repeated_no_progress_escalates(self):
  self.assertTrue(needs_escape(dict(retry=3,progress_m=.0003)))
  self.assertFalse(needs_escape(dict(retry=2,progress_m=0)))
  self.assertFalse(needs_escape(dict(retry=3,progress_m=.1)))
