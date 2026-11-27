import unittest
from unittest.mock import patch
import test_cyclic_recovery as cyclic
import test_preclose_alignment as alignment
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure
from grasp_runtime.autograsp.preclose_alignment import align_before_close
from grasp_runtime.autograsp.ros_backend import RosBackend
class FreshObservationTests(unittest.TestCase):
 def test_overhead_finish_invokes_camera_before_contact(self):
  from types import SimpleNamespace
  b=SimpleNamespace(candidate={'strategy':'overhead'})
  with patch('grasp_runtime.autograsp.preclose_alignment.align_before_close',side_effect=RuntimeError('camera-called')) as f:
   with self.assertRaisesRegex(RuntimeError,'camera-called'):RosBackend._finish_grasp(b,None,None)
   f.assert_called_once_with(b)
 def test_propagated_anchor_cannot_be_fresh_confirmation(self):
  helper=alignment.PrecloseTests();b,c=helper.backend(iter([]))
  b._observe_tracking=lambda:({'snapshot':{}},{'center_m':[0,0,0],'depth_measured':False})
  with self.assertRaises(AttemptFailure):align_before_close(b)
  self.assertTrue(b.pose_locked);self.assertFalse(c)
 def test_occluded_initial_approach_resets_without_four_blind_rotations(self):
  helper=cyclic.CyclicTests();b,c=helper.backend();attempts=[]
  def execute(candidate,scene):
   attempts.append(1)
   if len(attempts)==1:raise AttemptFailure(Failure.OCCLUDED,'surface_moved')
  b.execute_candidate=execute
  result=helper.run_backend(b)
  self.assertEqual(result['status'],'grasp_verified');self.assertEqual(c,['reset','stop'])
 def test_occluded_rotation_breaks_into_reobservation(self):
  helper=cyclic.CyclicTests();b,c=helper.backend();attempts=[]
  def execute(candidate,scene):
   attempts.append(1)
   if len(attempts)==1:raise AttemptFailure(Failure.MISALIGNED)
  b.execute_candidate=execute
  def rotate(n):c.append(n);raise AttemptFailure(Failure.OCCLUDED)
  b.rotate_only_retry=rotate
  helper.run_backend(b)
  self.assertEqual(c,['begin',1,'reset','stop'])
