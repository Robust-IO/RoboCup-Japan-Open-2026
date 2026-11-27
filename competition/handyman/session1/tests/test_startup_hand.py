import unittest,math
from types import SimpleNamespace
from pathlib import Path
from startup_hand import prepare
from run_sessions12 import task_command
class StartupHandTests(unittest.TestCase):
 def runtime(self,opening):
  self.moves=[]
  return SimpleNamespace(sample=lambda **k:None,state=lambda:{'joints':[0,0,0,-math.pi/2,0,opening]},move_joints=lambda *a,**k:self.moves.append(a) or {})
 def test_already_open_no_repeated_six_second_motion(self):
  r=prepare(self.runtime(.64999));self.assertFalse(r['motion_sent']);self.assertFalse(self.moves)
 def test_closed_empty_hand_still_opens(self):
  r=prepare(self.runtime(.1));self.assertTrue(r['motion_sent']);self.assertEqual(self.moves[0][0][5],.65)
 def test_invalid_feedback_not_treated_as_open(self):
  with self.assertRaises(ValueError):prepare(self.runtime(float('nan')))
  self.assertFalse(self.moves)
 def test_noninitial_arm_not_moved(self):
  rt=self.runtime(.65);rt.state=lambda:{'joints':[.4,1,0,0,0,.65]}
  with self.assertRaises(ValueError):prepare(rt)
  self.assertFalse(self.moves)
 def test_all_six_fresh_tasks_reuse_runtime_for_preparation(self):
  for index in range(6):self.assertIn('--prepare-initial-empty',task_command(index,Path('/config'),Path('/output')))

 def test_resume_never_opens_held_gripper(self):
  self.assertNotIn('--prepare-initial-empty',task_command(2,Path('/config'),Path('/output'),fresh=False))
