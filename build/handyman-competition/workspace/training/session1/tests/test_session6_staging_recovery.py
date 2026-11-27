import unittest
from types import SimpleNamespace
from threading import Event
from staging_view_recovery import reacquire,prepare_with_recovery
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure
from grasp_runtime.autograsp.planner import Policy
from grasp_runtime.autograsp.lift_headroom import lift_goal
from grasp_runtime.autograsp.grip_confirmation import accepted

class RecoveryTests(unittest.TestCase):
 def stages(self):
  events=[];heads=[];b=SimpleNamespace(identity_head_goal=(0.,-.5),holding_state=lambda:'empty',record=events.append)
  def check(c):
   if c.is_set():raise RuntimeError('task_cancelled')
  s=SimpleNamespace(grasp_backend=b,runtime=SimpleNamespace(face_source=heads.append),check=check,target='filled_ketchup')
  s.reacquire_after_staging=lambda c:reacquire(s,c)
  return s,heads,events
 def test_clipped_box_head_hint_before_other_views(self):
  s,heads,_=self.stages()
  def observe(t):
   if len(heads)==1:
    s.grasp_backend.identity_head_hint=(0.,-.25)
    raise AttemptFailure(Failure.OCCLUDED,'target_geometry_clipped_recenter_required')
   return 'complete-scene'
  s.grasp_backend.observe_stopped=observe
  self.assertEqual(reacquire(s,Event()),'complete-scene');self.assertEqual(heads,[(0.,-.5),(0.,-.25)])
 def test_exhaustion_recovers_instead_of_stopping(self):
  s,heads,events=self.stages();calls=[];s.grasp_backend.observe_stopped=lambda target:'scene'
  def prepare(*a):
   calls.append(1)
   if len(calls)==1:raise AttemptFailure(Failure.NO_CANDIDATE,'grasp_staging_views_exhausted_requires_new_observation')
   return 'ready'
  self.assertEqual(prepare_with_recovery(s,{},Event(),prepare),'ready');self.assertEqual(len(calls),2)
  self.assertEqual(heads,[(0.,-.15)]);self.assertFalse(events[0]['task_aborted'])
 def test_cancel_exits_recovery(self):
  s,heads,_=self.stages();cancel=Event()
  def prepare(*a):cancel.set();raise AttemptFailure(Failure.NO_CANDIDATE)
  with self.assertRaisesRegex(RuntimeError,'task_cancelled'):prepare_with_recovery(s,{},cancel,prepare)
  self.assertEqual(heads,[])
 def test_unrelated_error_is_not_retried(self):
  s,_,_=self.stages()
  def fail(*a):raise ValueError('invalid_geometry')
  with self.assertRaisesRegex(ValueError,'invalid_geometry'):prepare_with_recovery(s,{},Event(),fail)

class HeadroomTests(unittest.TestCase):
 def test_neck_high_pose_uses_remaining_travel(self):
  for lift in [.6492253274,.6660216511]:
   goal,d=lift_goal([lift,0,0,0,0,.4],Policy(lift_check_m=.02))
   self.assertAlmostEqual(goal[0],.69);self.assertGreaterEqual(d,.02)
 def test_normal_low_pose_keeps_eight_cm(self):
  goal,d=lift_goal([.4,0,0,0,0,.4],Policy());self.assertAlmostEqual(d,.08)
 def test_insufficient_travel_is_not_accepted(self):
  with self.assertRaises(ValueError):lift_goal([.68,0,0,0,0,.4],Policy(lift_check_m=.02))
 def test_short_lift_requires_explicit_minimum_and_blocked_reclose(self):
  e=dict(verified=True,grip_retained=True,observations=3,basis='lift_and_reclose_joint_feedback',lift_distance_m=.024,reclose_blocked=True)
  self.assertFalse(accepted(e));e['minimum_lift_m']=.02;self.assertTrue(accepted(e))
  e['reclose_blocked']=False;self.assertFalse(accepted(e))
  e['reclose_blocked']=True;e['minimum_lift_m']=.001;self.assertFalse(accepted(e))
