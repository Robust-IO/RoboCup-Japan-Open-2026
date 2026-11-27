import copy,json,tempfile,unittest
from pathlib import Path
from motion_speed import load_profile,arm_duration,PROFILE_PATH
class ArmMaxTests(unittest.TestCase):
 def test_all_measured_profiles_use_unity_maxima_empty_and_holding(self):
  for name in ('balanced_v1','carry_fast_v1','efficiency_v1'):
   p=load_profile(name)
   for holding in (False,True):
    self.assertEqual(arm_duration(p,'arm',[0]*6,[.3,0,0,0,0,0],holding=holding),2.)
    for joint in range(1,5):
     goal=[0.]*6;goal[joint]=1.
     self.assertEqual(arm_duration(p,'arm',[0]*6,goal,holding=holding),1.)
    self.assertEqual(arm_duration(p,'hand',[0]*6,[0,0,0,0,0,.65],holding=holding),6.)
 def test_short_motion_not_held_to_legacy_1_2_seconds(self):
  self.assertEqual(arm_duration(load_profile('balanced_v1'),'arm',[0]*6,[.015,0,0,0,0,0]),.1)
 def test_mixed_axes_never_exceed_limits(self):
  for name in ('balanced_v1','carry_fast_v1','efficiency_v1'):
   p=load_profile(name)
   for goal in ([.3,1.,-.4,.6,1.7,.4],[.013,.04,0,0,.08,.65]):
    dt=arm_duration(p,'arm',[0]*6,goal,holding=True)
    self.assertLessEqual(abs(goal[0])/dt,.15+1e-10)
    self.assertTrue(all(abs(x)/dt<=1.+1e-10 for x in goal[1:5]))
 def test_above_maximum_is_rejected(self):
  for key,v in [('empty_lift_mps',.151),('holding_lift_mps',.151),('empty_angular_rps',1.01),('holding_angular_rps',1.01)]:
   d=json.loads(PROFILE_PATH.read_text());d['profiles']['balanced_v1']['arm'][key]=v
   with tempfile.TemporaryDirectory() as tmp:
    path=Path(tmp)/'bad.json';path.write_text(json.dumps(d))
    with self.assertRaises(ValueError):load_profile('balanced_v1',path)
