import unittest
from types import SimpleNamespace
from unittest.mock import patch
from grasp_runtime.autograsp.contact_observation import observe_current
from grasp_runtime.autograsp.coordinator import AttemptFailure,LiftEvidence
from grasp_runtime.autograsp.recovery import Failure
from grasp_runtime.autograsp.cyclic_recovery import CyclicCoordinator
from grasp_runtime.autograsp.planner import Policy

class ContactViewTests(unittest.TestCase):
 def backend(self):
  heads=[];events=[]
  b=SimpleNamespace(pose_locked=True,static_precontact_anchor=True,record=events.append,
                    rt=SimpleNamespace(face_source=heads.append),check_task_cancelled=lambda:None)
  return b,heads
 def test_local_head_retry_requires_real_depth_and_restores_flags(self):
  b,heads=self.backend();calls=[]
  def observe():
   self.assertFalse(b.pose_locked);self.assertFalse(b.static_precontact_anchor);calls.append(1)
   if len(calls)==1:raise AttemptFailure(Failure.OCCLUDED,'insufficient_geometry_colour_correspondences')
   return {},dict(depth_measured=True)
  b._observe_tracking=observe
  self.assertTrue(observe_current(b)[1]['depth_measured']);self.assertEqual(heads,[(0.,-.3)])
  self.assertTrue(b.pose_locked);self.assertTrue(b.static_precontact_anchor)
 def test_never_accepts_stale_anchor(self):
  b,heads=self.backend();b._observe_tracking=lambda:({},dict(depth_measured=False))
  with self.assertRaises(AttemptFailure):observe_current(b)
  self.assertEqual(len(heads),3)
 def test_cancel_does_not_start_head_retry(self):
  b,heads=self.backend()
  def cancel():raise RuntimeError('task_cancelled')
  b.check_task_cancelled=cancel
  with self.assertRaisesRegex(RuntimeError,'task_cancelled'):observe_current(b)
  self.assertEqual(heads,[]);self.assertTrue(b.pose_locked)

class StagingHandoffTests(unittest.TestCase):
 def test_no_partial_resample_between_staging_and_selection(self):
  scene=object();candidate={'id':'neck','strategy':'side'};used=[]
  b=SimpleNamespace(pending_grasp_scene=scene,holding_state=lambda:'empty',check_task_cancelled=lambda:None,
    observe_stopped=lambda target:self.fail('accepted staging scene must not be replaced'),stop=lambda:None,
    execute_candidate=lambda c,s:used.append(s),candidate=candidate,
    verify_lift=lambda:LiftEvidence(True,False,True,3,'lift_and_reclose_joint_feedback',.08,True))
  def planner(s,h,p):self.assertIs(s,scene);return dict(selected=candidate)
  with patch('grasp_runtime.autograsp.cyclic_recovery.select_template',return_value=(Policy(),{})),patch('grasp_runtime.autograsp.cyclic_recovery.plan',side_effect=planner):
   result=CyclicCoordinator(b,None,Policy(),lambda e:None).run('filled_ketchup')
  self.assertEqual(result['status'],'grasp_verified');self.assertEqual(used,[scene]);self.assertIsNone(b.pending_grasp_scene)
