import unittest
from unittest.mock import patch
import numpy as np
from grasp_runtime.autograsp.near_side_plan import replan_side
from grasp_runtime.autograsp.planner import Policy
class NearSideTests(unittest.TestCase):
 def test_refresh_keeps_foreign_surface_and_forces_neutral_side(self):
  target=np.array([[.8,0,.45],[.8,.01,.45],[.8,-.01,.45]])
  points=np.vstack([target,[[.83,.07,.45],[.8,0,.4]]])
  with patch('grasp_runtime.autograsp.near_side_plan.plan',return_value={'selected':None}) as planner:
   replan_side(target,points,[0,0,.4],123,None,Policy())
   scene,_,policy=planner.call_args.args
   np.testing.assert_allclose(scene.target,target)
   np.testing.assert_allclose(scene.environment,points[-2:])
   self.assertEqual(policy.allowed_strategies,('side',))
   self.assertTrue(policy.neutral_wrist_only)
 def test_blocked_close_range_plan_is_not_waived(self):
  target=np.array([[.8,0,.45]])
  with patch('grasp_runtime.autograsp.near_side_plan.plan',return_value={'selected':None,'rejected':{'collision':1}}):
   self.assertIsNone(replan_side(target,target,[0,0,.4],123,None,Policy())['selected'])
