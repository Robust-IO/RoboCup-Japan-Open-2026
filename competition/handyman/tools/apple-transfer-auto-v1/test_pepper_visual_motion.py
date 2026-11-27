import copy,unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from pepper_visual_motion import fit_rigid,NearAnchor

class VisualMotionTests(unittest.TestCase):
 def test_visible_radius_fit_keeps_current_surface_and_rejects_plane_or_clutter(self):
  from pepper_geometry import fit_visible_known_radius
  rng=np.random.default_rng(48);c=np.array([.74,.08,.53]);r=.0316
  az=rng.uniform(-.6,.6,300);el=rng.uniform(-.45,.45,300)
  p=c+r*np.c_[-np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)]
  p+=rng.normal(0,.00025,p.shape)
  measured,radius,proof=fit_visible_known_radius(np.r_[p,p[:20]+[.008,0,0]],r,c+[.003,-.002,.001])
  np.testing.assert_allclose(measured,c,atol=.0015);self.assertEqual(radius,r)
  self.assertFalse(proof['hidden_points_used']);self.assertGreater(proof['rejected_current_points'],0)
  plane=p.copy();plane[:,0]=c[0]-r
  with self.assertRaises(ValueError):fit_visible_known_radius(plane,r,c)
  with self.assertRaises(ValueError):fit_visible_known_radius(np.r_[p,p+[.012,0,0]],r,c)
 def test_rigid_motion_recovered_with_wrong_feature_matches(self):
  rng=np.random.default_rng(123);a=rng.uniform([.6,-.5,.45],[1.5,.5,1.3],(80,3))
  r=Rotation.from_euler('z',.04).as_matrix();t=np.array([-.27,-.033,.001]);b=a@r.T+t+rng.normal(0,.0007,a.shape)
  b[-18:]=rng.uniform([.5,-.5,.4],[1.5,.5,1.3],(18,3))
  actual,shift,proof=fit_rigid(a,b)
  np.testing.assert_allclose(actual,r,atol=.002);np.testing.assert_allclose(shift,t,atol=.002)
  self.assertGreaterEqual(proof['inliers'],60);self.assertFalse(proof['object_depth_measured'])
 def test_small_or_collinear_matches_are_not_a_motion_measurement(self):
  for n in (5,30):
   a=np.c_[np.linspace(0,1,n),np.zeros((n,2))]
   with self.subTest(n=n),self.assertRaises(ValueError):fit_rigid(a,a)
 def test_unrelated_clouds_do_not_create_a_target_prediction(self):
  rng=np.random.default_rng(42)
  with self.assertRaises(ValueError):fit_rigid(rng.normal(size=(40,3)),rng.normal(size=(40,3)))
 def test_old_epoch_or_frame_is_rejected_before_features(self):
  tf=dict(translation=[0,0,0],quaternion=[0,0,0,1])
  snapshot=dict(tracking_epoch='current',streams=dict(depth=dict(stamp_ns=1)),tracking_transforms=dict(odom_from_base_at_depth=tf))
  anchor=NearAnchor(dict(snapshot=snapshot),dict(stamp_ns=1,center_m=[1,0,.5]))
  other=copy.deepcopy(snapshot);other['tracking_epoch']='old';other['streams']['depth']['stamp_ns']=2
  with self.assertRaises(ValueError):anchor.predict(dict(snapshot=other),[1,0,.5],[1,0,.55])
  with self.assertRaises(ValueError):anchor.predict(dict(snapshot=snapshot),[1,0,.5],[1,0,.55])
 def test_stationary_frame_does_not_run_feature_matching_or_replace_depth(self):
  tf=dict(translation=[0,0,0],quaternion=[0,0,0,1])
  s=dict(tracking_epoch='same',streams=dict(depth=dict(stamp_ns=1)),tracking_transforms=dict(odom_from_base_at_depth=tf))
  anchor=NearAnchor(dict(snapshot=s),dict(stamp_ns=1,center_m=[1,0,.5]))
  new=copy.deepcopy(s);new['streams']['depth']['stamp_ns']=2
  with patch('pepper_visual_motion._features') as features:
   self.assertIsNone(anchor.predict(dict(snapshot=new),[1,0,.5],[1,0,.55]));features.assert_not_called()

if __name__=='__main__':unittest.main()
