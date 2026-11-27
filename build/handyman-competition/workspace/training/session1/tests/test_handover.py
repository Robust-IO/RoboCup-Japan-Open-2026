import unittest
import numpy as np
from handover import held_center,presenting_joints,delivery_evidence
from grasp_runtime.autograsp.kinematics import forward

class HandoverTests(unittest.TestCase):
 def context(self):return dict(frame='hand_palm_link',vertices_palm_m=[[-.06,-.07,.01],[.08,.06,.16]])
 def test_presentation_tracks_height_without_opening(self):
  c=self.context()
  for height in (.8,1.,1.1):
   q=presenting_joints(c,height,.42);p,r=forward(q)
   self.assertAlmostEqual((p+r@held_center(c))[2],height)
   self.assertEqual(q[5],.42);self.assertGreaterEqual(q[0],0);self.assertLessEqual(q[0],.69)
 def test_far_height_rejected(self):
  with self.assertRaises(ValueError):presenting_joints(self.context(),3.,.42)
 def test_distance_is_3d_and_hold_is_required(self):
  for center,holding,ready in [([0,0,.1],True,True),([0,0,.25],True,False),([0,0,.1],False,False)]:
   p=delivery_evidence(center,[0,0,0],dict(holding_verified=holding),target='dog_doll')
   self.assertEqual(p['delivery_ready'],ready);self.assertFalse(p['release_sent']);self.assertFalse(p['official_success'])
 def test_nonfinite_geometry_rejected(self):
  with self.assertRaises(ValueError):delivery_evidence([0,float('nan'),0],[0,0,0],{},target='dog_doll')

class LiveHandoverTests(unittest.TestCase):
 def run_present(self,cancelled=False):
  from unittest.mock import patch
  from types import SimpleNamespace
  from pathlib import Path
  from tempfile import TemporaryDirectory
  from threading import Event
  from handover_live import HandoverLive
  rt=SimpleNamespace();q=[0.,0.,0.,-1.57,0.,.42];moves=[]
  rt.sample=lambda **k:None
  rt.state=lambda:dict(joints=list(q),poses={'map':[1.,0.,3.14]})
  def move(goal,label):moves.append((list(goal),label));q[:]=goal
  rt.move_joints=move
  motion=SimpleNamespace(adjust=lambda *a,**k:self.fail('already aligned must not drive'))
  cancel=Event()
  if cancelled:cancel.set()
  def check(c):
   if c.is_set():raise RuntimeError('cancelled')
  with TemporaryDirectory() as tmp:
   stage=SimpleNamespace(runtime=rt,motion=motion,grasp_backend=SimpleNamespace(record=lambda event:None),context=HandoverTests().context(),target='dog_doll',output=Path(tmp),check=check,
       config={'handover':dict(delivery_point_map_m=[0.,0.,1.],estimated_center_tolerance_m=.2)})
   live=HandoverLive(stage,map_point=lambda *a:None,rotation=lambda *a:None)
   live.last_visual=dict(visual_confirmation=True)
   live.observation=lambda:({},np.array([.03,0.,1.]))
   with patch('handover_live.HoldObserver',return_value=lambda label:dict(holding_verified=True,visual_confirmation=True)), patch('handover_reclose.confirm',return_value=.42) as reclose:
    if cancelled:
     with self.assertRaises(RuntimeError):live.present({},cancel)
     self.assertEqual(moves,[]);return
    proof=live.present({},cancel)
    reclose.assert_called_once()
    self.assertTrue(proof['fresh_delivery_reclose_verified'])
   self.assertTrue(proof['delivery_ready']);self.assertFalse(proof['release_sent'])
   self.assertTrue(moves);self.assertTrue(all(goal[5]==.42 for goal,label in moves))
 def test_holding_preserved_and_no_extra_drive(self):self.run_present()
 def test_cancellation_before_motion(self):self.run_present(True)

class SceneExtractionTests(unittest.TestCase):
 def test_parent_transform_and_stripped_unrelated_transform(self):
  from prepare_session3_handover import scene_point
  text='''--- !u!4 &99 stripped
Transform:
  m_CorrespondingSourceObject: {fileID: 3}
--- !u!1 &1
GameObject:
  m_Name: Moderator
  m_TagString: Moderator
--- !u!4 &2
Transform:
  m_GameObject: {fileID: 1}
  m_LocalRotation: {x: 0, y: 0, z: 0, w: 1}
  m_LocalPosition: {x: 1, y: 0, z: 2}
  m_LocalScale: {x: 1, y: 1, z: 1}
  m_Father: {fileID: 0}
--- !u!1 &3
GameObject:
  m_Name: DeliveryPosition
--- !u!4 &4
Transform:
  m_GameObject: {fileID: 3}
  m_LocalRotation: {x: 0, y: 0, z: 0, w: 1}
  m_LocalPosition: {x: 0, y: 1, z: 0.3}
  m_LocalScale: {x: 0.01, y: 0.01, z: 0.01}
  m_Father: {fileID: 2}
'''
  np.testing.assert_allclose(scene_point(text),[2.3,-1,1])
