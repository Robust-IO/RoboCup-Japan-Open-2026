import unittest
from types import SimpleNamespace
from unittest.mock import patch
from carry_drop_recovery import enabled,stable_pair,checkpoint
class DropTests(unittest.TestCase):
 def stage(self,n=6):return SimpleNamespace(config={'session_profile':{'session_number':n},'policy':{'carry_drop_recovery':True}},context={},motion=SimpleNamespace(stage='navigate_destination'))
 def frame(self,stamp,x=0,target='filled_ketchup'):
  return dict(depth_stamp_ns=stamp,floor_visible=True,floor_targets=[dict(target=target,center_map_m=[x,0,.07])])
 def test_only_enabled_session256_carry(self):
  self.assertTrue(enabled(self.stage()))
  for n in (1,3,4):self.assertFalse(enabled(self.stage(n)))
  for n in (2,5,6):self.assertTrue(enabled(self.stage(n)))
  s=self.stage(3);s.motion.stage="navigate_moderator";self.assertFalse(enabled(s))
  s=self.stage();s.drop_recovery_active=True;self.assertFalse(enabled(s))
  s=self.stage();s.motion.stage='grasp';self.assertFalse(enabled(s))
 def test_two_fresh_stable_frames_required(self):
  self.assertIsNotNone(stable_pair(self.frame(1),self.frame(2,.02),'filled_ketchup'))
  for f in [self.frame(1),self.frame(2,.2),self.frame(2,target='salt')]:self.assertIsNone(stable_pair(self.frame(1),f,'filled_ketchup'))
  f=self.frame(2);f['floor_visible']=False;self.assertIsNone(stable_pair(self.frame(1),f,'filled_ketchup'))
 def test_held_checkpoint_never_opens_or_searches(self):
  s=self.stage();s.grasp_backend=SimpleNamespace(record=lambda e:None);s.runtime=object();s.check=lambda c:None
  with patch('handover_reclose.confirm',return_value=.33):self.assertFalse(checkpoint(s,object()))
  self.assertEqual(s.context['holding_opening_rad'],.33)
 def test_uncertain_grip_does_not_authorize_floor_grasp(self):
  from grasp_runtime.autograsp.coordinator import AttemptFailure
  from grasp_runtime.autograsp.recovery import Failure
  s=self.stage();s.grasp_backend=SimpleNamespace(record=lambda e:None);s.runtime=object();s.check=lambda c:None
  with patch('handover_reclose.confirm',side_effect=AttemptFailure(Failure.HOLD_UNCERTAIN)):
   with self.assertRaises(AttemptFailure):checkpoint(s,object())
