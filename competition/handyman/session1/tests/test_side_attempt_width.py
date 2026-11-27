import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from grasp_runtime.autograsp import planner

class SideAttemptWidthTests(unittest.TestCase):
 def scene(self):return SimpleNamespace(target=np.zeros((60,3)),stamp_ns=1,validate=lambda p:None)
 def evaluate(self,row,*a):
  strategy=row['strategy']
  return dict(id=strategy,strategy=strategy,wrist_roll_rad=0.,grasp_point_base_m=[0,0,0],score=1,observed_width_m=row['width']),None
 def test_local_width_boundary_not_whole_object_width(self):
  for width,expected in ((.09398,'side'),(.095,'side'),(.09501,'overhead')):
   rows=[{'strategy':'side','width':width},{'strategy':'overhead','width':.04}]
   with self.subTest(width=width),patch.object(planner,'minimum_horizontal_width',return_value=.137),patch.object(planner,'generate',return_value=rows),patch.object(planner,'evaluate',side_effect=self.evaluate):
    result=planner.plan(self.scene(),None)
    self.assertEqual(result['selected']['strategy'],expected)
    self.assertFalse(result['whole_object_width_gate_enabled'])
    self.assertFalse(result['side_excluded_by_width'])
  self.assertEqual(planner.Policy().jaw_inner_width_m,.093026)
 def test_local_candidate_still_requires_physical_evaluation(self):
  rows=[{'strategy':'side','width':.045}]
  with patch.object(planner,'minimum_horizontal_width',return_value=.2),patch.object(planner,'generate',return_value=rows),patch.object(planner,'evaluate',return_value=(None,'approach_hits_surroundings')) as evaluate:
   result=planner.plan(self.scene(),None)
   evaluate.assert_called_once();self.assertIsNone(result['selected'])
   self.assertEqual(result['rejected'],{'approach_hits_surroundings':1})
 def test_wide_body_with_narrow_band_generates_side_candidates(self):
  # A 16cm-wide body and a narrower upper section: not one bbox-sized grasp.
  x,y,z=np.meshgrid(np.linspace(.6,.8,13),np.linspace(-.08,.08,17),np.linspace(.02,.08,8))
  body=np.c_[x.ravel(),y.ravel(),z.ravel()]
  x,y,z=np.meshgrid(np.linspace(.65,.75,9),np.linspace(-.025,.025,11),np.linspace(.09,.16,10))
  neck=np.c_[x.ravel(),y.ravel(),z.ravel()]
  scene=planner.Scene(np.r_[body,neck],np.empty((0,3)),np.zeros(3),1)
  self.assertGreater(planner.minimum_horizontal_width(scene.target),.095)
  rows=planner.generate(scene,planner.Policy())
  side=[r for r in rows if r['strategy']=='side']
  self.assertTrue(side);self.assertTrue(any(r['width']<.06 for r in side))
  with patch.object(planner,'evaluate',side_effect=self.evaluate):
   result=planner.plan(scene,None)
  self.assertEqual(result['selected']['strategy'],'side')
 def test_invalid_local_width_does_not_authorize_attempt(self):
  with patch.object(planner,'generate',return_value=[dict(strategy='side',width=float('nan'))]),patch.object(planner,'evaluate') as evaluate:
   result=planner.plan(self.scene(),None)
   evaluate.assert_not_called();self.assertIsNone(result['selected'])
