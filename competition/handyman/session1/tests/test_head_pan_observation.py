import unittest,math
from types import SimpleNamespace
from threading import Event
from furniture_reobservation import observe_furniture
from staging_view_recovery import reacquire
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure

class HeadPanTests(unittest.TestCase):
 def fixture(self):
  heads=[]
  def check(c):
   if c.is_set():raise RuntimeError('task_cancelled')
  b=SimpleNamespace(record=lambda r:None,table={'axis_aligned_bounds':{'center':{'x':1,'y':0}}},identity_head_goal=(0.,0.))
  s=SimpleNamespace(state=lambda c:{'poses':{'map':[0,0,0]}},check=check,candidate_selector=None,
      grasp_backend=b,source_room='room',source_id='table',target='can',
      environment={'rooms':{'room':{'region':[[-2,-2],[2,-2],[2,2],[-2,2]]}}},
      face_source_furniture=lambda *a:None,runtime=SimpleNamespace(face_source=lambda h:heads.append(h)))
  return s,heads
 def test_right_side_success_stops_before_more_tilts(self):
  s,heads=self.fixture()
  def observe(*a,head=None):
   heads.append(head)
   if head is not None and head[0]<0:return {'confirmed':True}
   raise AttemptFailure(Failure.OCCLUDED,'not_visible')
  s._observe_target_once=observe
  self.assertTrue(observe_furniture(s,{},Event())['confirmed'])
  self.assertEqual(len(heads),3)
  self.assertGreater(heads[1][0],0);self.assertLess(heads[2][0],0)
 def test_each_main_tilt_gets_both_sides_once(self):
  s,heads=self.fixture()
  def observe(*a,head=None):heads.append(head);return {'confirmed':False}
  s._observe_target_once=observe
  self.assertFalse(observe_furniture(s,{},Event())['confirmed'])
  for tilt in (0.,-math.pi/6,-.8):
   for pan in (math.pi/6,-math.pi/6):self.assertIn((pan,tilt),heads)
  self.assertEqual(len(heads),len(set(heads)))
 def test_staging_can_reacquire_from_side(self):
  s,heads=self.fixture()
  def observe(target):
   if heads[-1][0]<0:return 'scene'
   raise AttemptFailure(Failure.OCCLUDED,'not_visible')
  s.grasp_backend.observe_stopped=observe
  self.assertEqual(reacquire(s,Event()),'scene');self.assertEqual(len(heads),3)
 def test_cancel_prevents_head_motion(self):
  s,heads=self.fixture();cancel=Event();cancel.set()
  with self.assertRaisesRegex(RuntimeError,'task_cancelled'):reacquire(s,cancel)
  self.assertFalse(heads)
