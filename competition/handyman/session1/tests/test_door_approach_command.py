import unittest,math
from door_approach_heading import approach_command
class DoorCommandTests(unittest.TestCase):
 def test_recorded_pose_centers_without_opposite_turn(self):
  v=approach_command([4.5107,5.4544,-1.53465],[4.596815,5.465263,-math.pi/2],True)
  self.assertLess(v[2],0);self.assertGreater(v[1],0);self.assertLessEqual(math.hypot(*v[:2]),.12)
 def test_large_heading_error_turns_before_translation(self):
  v=approach_command([.12,0,2.8],[0,0,1.57],True);self.assertEqual(v[:2],(0.,0.))
 def test_furniture_and_far_approach_unchanged(self):
  self.assertIsNone(approach_command([.1,0,0],[0,0,0],False))
  self.assertIsNone(approach_command([.3,0,0],[0,0,0],True))
