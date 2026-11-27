import unittest
import numpy as np
from candidate_selection import CandidateSelector
from carry_clearance import CarryClearance,ProgressWatch
from hand_hold_evidence import compare
from handover import delivery_evidence

class CarryTests(unittest.TestCase):
 def scene(self):
  grid=np.ones((120,120),bool);grid[:2]=False;grid[-2:]=False;grid[:,:2]=False;grid[:,-2:]=False
  grid[40:80,70:75]=False
  selector=CandidateSelector(grid,.025,[0.,0.,0.],.22,{})
  return CarryClearance(selector,np.array([[.36,-.08,.3],[.4,.08,.4]]))
 def test_retreat_before_turn_near_furniture(self):
  guard=self.scene();pose=[1.32,1.5,0.]
  self.assertFalse(guard.turn_clear(pose));plan=guard.escape(pose)
  self.assertLess(plan['delta'][0],0.);self.assertEqual(plan['delta'][2],0.)
  self.assertTrue(guard.turn_clear(plan['pose']))
 def test_no_retreat_if_clear(self):self.assertEqual(self.scene().escape([.8,1.5,0.])['distance_m'],0.)
 def test_no_escape_through_wall(self):
  g=self.scene();g.selector.free[:]=False;g.clearance[:]=0.
  with self.assertRaises(ValueError):g.escape([1.32,1.5,0.])
 def test_yaw_wiggle_is_not_progress(self):
  w=ProgressWatch()
  for t in range(12):self.assertFalse(w.stalled([0,0,(-1)**t*.5],t,[1,0,0]))
  self.assertTrue(w.stalled([0,0,0],12,[1,0,0]))
 def test_real_progress_resets_window(self):
  w=ProgressWatch();self.assertFalse(w.stalled([0,0,0],0,[1,0,0]))
  self.assertFalse(w.stalled([.1,0,0],11,[1,0,0]));self.assertFalse(w.stalled([.1,0,0],20,[1,0,0]))
 def test_changed_background_not_hold(self):
  rng=np.random.default_rng(7);ref=rng.integers(0,255,(50,50,3),dtype=np.uint8);mask=np.ones((50,50),np.uint8)
  self.assertTrue(compare(ref,ref.copy(),mask)['matched'])
  self.assertFalse(compare(ref,np.full_like(ref,120),mask)['matched'])
 def test_grip_only_cannot_authorize_handover(self):
  p=delivery_evidence([0,0,0],[0,0,0],dict(holding_verified=True,visual_confirmation=False),target='dog_doll',require_visual=True)
  self.assertFalse(p['delivery_ready'])
 def test_turn_monitor_near_goal_accepts_converging_yaw(self):
  w=ProgressWatch()
  for t in range(20):self.assertFalse(w.stalled([0,0,2.-t*.1],t,[0,0,0]))
 def test_removed_lift_bypass_in_strict_mode(self):
  from unittest.mock import patch
  from test_hold_and_approach import Tests
  helper=Tests();helper.setUp()
  try:
   observer,b=helper.hold();observer.context.update(require_visual_carry_checks=True,lift_evidence={'basis':'lift_and_reclose_joint_feedback'})
   b._cloud=lambda i:(np.empty((0,3)),np.empty((0,3)),np.array([],int))
   with patch('hand_hold_evidence.HandHoldEvidence',side_effect=ValueError('no_visual_evidence')):
    with self.assertRaises(ValueError):observer('handover-check')
  finally:helper.doCleanups()
