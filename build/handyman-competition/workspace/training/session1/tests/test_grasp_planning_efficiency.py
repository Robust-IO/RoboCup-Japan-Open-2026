import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from grasp_runtime.autograsp.contact import small_height_correction
from grasp_runtime.autograsp.planner import plan,Policy
class EfficiencyTests(unittest.TestCase):
 def test_recorded_tiny_deficit_is_corrected_not_waived(self):
  dz=small_height_correction({'reason':'actual_closing_hand_too_low','clearance_m':.004981911551503693},.005)
  self.assertAlmostEqual(dz,.001018088448496307)
  self.assertIsNone(small_height_correction({'reason':'actual_closing_hand_too_low','clearance_m':.001},.005))
  self.assertIsNone(small_height_correction({'reason':'material_outside_actual_open_gap'},.005))
 def test_stop_after_enough_fully_evaluated_candidates(self):
  scene=SimpleNamespace(target=np.array([[0,0,0],[.03,.03,.04]]),validate=lambda p:None,stamp_ns=1)
  rows=[dict(strategy='side',width=.03,idx=i) for i in range(30)]
  def evaluate(r,*args):
   i=r['idx'];return dict(id=str(i),strategy='side',wrist_roll_rad=0,grasp_point_base_m=[i*.02,0,0],score=i),None
  with patch('grasp_runtime.autograsp.planner.generate',return_value=rows),patch('grasp_runtime.autograsp.planner.evaluate',side_effect=evaluate) as e:
   result=plan(scene,None,Policy(feasible_budget=8))
   self.assertEqual(e.call_count,8);self.assertTrue(result['early_exit']);self.assertEqual(len(result['candidates']),8)
