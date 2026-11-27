import unittest
from navigation_turn_policy import ArrivalHeading,final_heading_command,final_pose_ready
class FinalHeadingTests(unittest.TestCase):
 def test_recorded_75mm_pose_finishes_without_nav2_handoff(self):
  goal=[1.787694871675014,-.020209843066725408,.4722355600495447]
  pose=[1.7863172524166282,.05433801177503018,.4350860110103384]
  arrival=ArrivalHeading();arrival.select(pose,goal,[],controller_rotating=True)
  self.assertTrue(arrival.arrived)
  self.assertEqual(final_heading_command(pose,goal),(0.,0.,0.))
  self.assertTrue(final_pose_ready(pose,goal))
 def test_incomplete_turn_still_rotates_correct_direction(self):
  self.assertGreater(final_heading_command([0,0,.25],[0,0,.47])[2],0)
  self.assertFalse(final_pose_ready([0,0,.25],[0,0,.47]))
 def test_far_pose_cannot_complete(self):
  self.assertFalse(final_pose_ready([.2,0,0],[0,0,0]))
