import unittest
from types import SimpleNamespace
from grasp_runtime.autograsp.ros_backend import RosBackend
from grasp_runtime.autograsp.coordinator import AttemptFailure
class ClosureFreshTests(unittest.TestCase):
 def test_callback_cannot_take_locked_snapshot(self):
  b=SimpleNamespace(pose_locked=True,static_precontact_anchor=True)
  def observe():
   self.assertFalse(b.pose_locked);self.assertFalse(b.static_precontact_anchor)
   return {'snapshot':{}},{'depth_measured':True}
  b._observe_tracking=observe
  self.assertTrue(RosBackend._observe_for_closure(b)[1]['depth_measured']);self.assertTrue(b.pose_locked)
 def test_stale_result_rejected(self):
  b=SimpleNamespace(pose_locked=True,static_precontact_anchor=True,_observe_tracking=lambda:({}, {'depth_measured':False}))
  with self.assertRaises(AttemptFailure):RosBackend._observe_for_closure(b)
  self.assertTrue(b.pose_locked)
