import unittest
from types import SimpleNamespace
from grasp_runtime.autograsp.combined_pregrasp import prepare_arm
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure
class CombinedTests(unittest.TestCase):
 def backend(self):
  self.calls=[];self.paths=[]
  return SimpleNamespace(rt=SimpleNamespace(state=lambda:{'joints':[0,0,0,-1.57,0,.65]}),_sample_for_path=lambda:None,
   _check_open_path=lambda path:self.paths.append(path),_arm=lambda goal,label:self.calls.append(label),record=lambda e:None)
 def test_clear_full_path_is_one_move(self):
  b=self.backend();prepare_arm(b,[.5,0,0,-1.57,0,.65],[.5,-1.7,0,.129,0,.65])
  self.assertEqual(self.calls,['generic-combined-raise-orient']);self.assertGreater(len(self.paths[0]),40)
 def test_blocked_combination_uses_old_sequence(self):
  b=self.backend()
  def blocked(path):raise AttemptFailure(Failure.APPROACH_COLLISION,'table')
  b._check_open_path=blocked;prepare_arm(b,[.5,0,0,-1.57,0,.65],[.5,-1.7,0,.129,0,.65])
  self.assertEqual(self.calls,['generic-raise-before-orient','generic-orient-open'])
 def test_execution_failure_does_not_replay(self):
  b=self.backend()
  def fail(goal,label):self.calls.append(label);raise RuntimeError('feedback_lost')
  b._arm=fail
  with self.assertRaisesRegex(RuntimeError,'feedback_lost'):prepare_arm(b,[.5,0,0,-1.57,0,.65],[.5,-1.7,0,.129,0,.65])
  self.assertEqual(len(self.calls),1)
