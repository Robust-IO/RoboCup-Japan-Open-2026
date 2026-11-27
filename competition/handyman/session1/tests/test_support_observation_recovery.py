import unittest
from types import SimpleNamespace
from grasp_runtime.autograsp.support_observation_recovery import observe
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure
class SupportRecoveryTests(unittest.TestCase):
 def backend(self):
  self.heads=[];return SimpleNamespace(check_task_cancelled=lambda:None,holding_state=lambda:'empty',record=lambda e:None,rt=SimpleNamespace(face_source=self.heads.append,pump=lambda t:None))
 def test_plane_failure_then_identity_miss_then_new_view(self):
  b=self.backend();rows=iter([ValueError('dominant_table_plane_not_established'),AttemptFailure(Failure.OCCLUDED,'identity_missing'),'scene'])
  def once(t):
   v=next(rows)
   if isinstance(v,Exception):raise v
   return v
  self.assertEqual(observe(b,'dog',once),'scene');self.assertEqual(len(self.heads),2);self.assertFalse(b.static_precontact_anchor)
 def test_unrelated_failure_not_swallowed(self):
  b=self.backend()
  def once(t):raise ValueError('bad_calibration')
  with self.assertRaisesRegex(ValueError,'bad_calibration'):observe(b,'dog',once)
  self.assertFalse(self.heads)
 def test_repeated_failure_remains_cancellable(self):
  b=self.backend();calls=[0]
  def check():
   calls[0]+=1
   if calls[0]>4:raise RuntimeError('cancelled')
  b.check_task_cancelled=check
  def once(t):raise ValueError('dominant_table_plane_not_established')
  with self.assertRaisesRegex(RuntimeError,'cancelled'):observe(b,'dog',once)
