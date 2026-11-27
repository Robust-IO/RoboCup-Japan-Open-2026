import unittest
from unittest.mock import patch
import numpy as np
from grasp_runtime.autograsp.near_side_plan import replan_side
from grasp_runtime.autograsp.planner import Policy

class MembershipTests(unittest.TestCase):
 def test_current_match_is_target_not_environment_and_still_checked(self):
  template=np.array([[.5,0,.5],[.5,.02,.5]])
  matched=np.array([[.507,0,.5]])
  furniture=np.array([[.55,.2,.5]])
  points=np.concatenate((matched,furniture));seen=[]
  def plan(scene,*a):seen.append(scene);return dict(selected=None)
  with patch('grasp_runtime.autograsp.near_side_plan.plan',side_effect=plan):
   replan_side(template,points,[0,0,.4],1,None,Policy(),observed_target=matched)
  np.testing.assert_allclose(seen[0].environment,furniture)
  self.assertTrue(np.any(np.all(seen[0].target==matched[0],axis=1)))
 def test_without_current_match_does_not_expand_global_exclusion(self):
  t=np.array([[.5,0,.5]]);p=np.array([[.507,0,.5]]);seen=[]
  with patch('grasp_runtime.autograsp.near_side_plan.plan',side_effect=lambda s,*a:seen.append(s)):
   replan_side(t,p,[0,0,.4],1,None,Policy())
  np.testing.assert_allclose(seen[0].environment,p)
 def test_invalid_current_points_rejected(self):
  with self.assertRaises(ValueError):
   replan_side(np.array([[0,0,1]]),np.empty((0,3)),[0,0,0],1,None,Policy(),observed_target=np.array([[float('nan'),0,1]]))
