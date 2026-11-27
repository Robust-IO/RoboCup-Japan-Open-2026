import unittest,math,tempfile,json
from pathlib import Path
from unittest.mock import patch
from threading import Event
from types import SimpleNamespace
from continuous_scan import SweepProgress,bin_index
from session3_search_live import Session3Search

class ContinuousTests(unittest.TestCase):
 def test_wrap_crossing_counts_actual_rotation(self):
  p=SweepProgress(3.1);self.assertAlmostEqual(p.update(-3.1),2*math.pi-6.2)
  self.assertAlmostEqual(p.update(-2.9),2*math.pi-6.0)
 def test_localization_jump_not_counted_as_coverage(self):
  with self.assertRaises(ValueError):SweepProgress(0).update(2.)
 def test_bins_wrap_and_do_not_depend_on_world_heading(self):
  for i in range(12):self.assertEqual(bin_index(2.9+i*math.pi/6,2.9),i)
 def test_complete_moving_views_only_navigate_to_search_point(self):
  with tempfile.TemporaryDirectory() as tmp:
   s=Session3Search.__new__(Session3Search);s.options=dict(continuous_bins=12,head_tilts=[-.4,-.8]);s.points=[dict(x=0,y=0,yaw=0)];s.serial=0;s.output=Path(tmp);s.cfg={'search_plan':{}}
   moves=[];s.rt=SimpleNamespace(face_source=lambda head:None,sample=lambda **k:None,state=lambda:{'poses':{'map':[0,0,0]}});s.navigate=lambda *a:moves.append(a);s.capture=lambda *a:self.fail('negative valid images need no repeats')
   def run(*args,**kwargs):return 0.,[dict(angle_bin=i,usable=True,matches=[],unresolved=False) for i in range(12)],2*math.pi
   with patch('continuous_scan.ContinuousSweep') as cls:
    cls.return_value.run.side_effect=run
    result=s.continuous_search({'task':{'target':'bear_doll'}},Event())
   self.assertTrue(result['search_exhausted']);self.assertEqual(result['valid_views'],24);self.assertEqual(len(moves),1);self.assertEqual(moves[0][0],s.points[0])
 def test_low_candidate_requires_stopped_recheck(self):
  with tempfile.TemporaryDirectory() as tmp:
   s=Session3Search.__new__(Session3Search);s.options=dict(continuous_bins=12,head_tilts=[-.4]);s.points=[dict(x=0,y=0,yaw=0)];s.serial=0;s.output=Path(tmp);s.cfg={'search_plan':{}};turns=[]
   s.rt=SimpleNamespace(face_source=lambda h:None,sample=lambda **k:None,state=lambda:{'poses':{'map':[0,0,0]}});s.navigate=lambda *a:turns.append(a);stamp=iter([1,2])
   s.capture=lambda *a:dict(usable=True,matches=[],unresolved=False,stamp_ns=next(stamp))
   rows=[dict(angle_bin=i,usable=True,matches=[],unresolved=(i==4)) for i in range(12)]
   with patch('continuous_scan.ContinuousSweep') as cls:
    cls.return_value.run.return_value=(0.,rows,2*math.pi)
    result=s.continuous_search({'task':{'target':'bear_doll'}},Event())
   self.assertTrue(result['search_exhausted']);self.assertEqual(len(turns),3);self.assertTrue(turns[0][2].startswith('continuous-point-'));self.assertTrue(turns[1][2].startswith('scan-repair-'))

class CoverageMarginTests(unittest.TestCase):
 def test_only_near_bin_center_counts_for_coverage(self):
  from continuous_scan import covered_bin
  self.assertEqual(covered_bin(math.radians(24),0),1)
  self.assertIsNone(covered_bin(math.radians(16),0))
  self.assertEqual(covered_bin(math.radians(354),0),0)

class SweepCleanupTests(unittest.TestCase):
 def test_capture_exception_stops_velocity_and_retires_publisher(self):
  import sys
  from continuous_scan import ContinuousSweep
  class Twist:
   def __init__(self):self.angular=SimpleNamespace(z=0.)
  sent=[];destroyed=[];timer=[None];yaw=[0.]
  pub=SimpleNamespace(publish=lambda msg:sent.append(msg.angular.z),get_subscription_count=lambda:1)
  node=SimpleNamespace(create_publisher=lambda *a:pub,create_timer=lambda dt,fn:timer.__setitem__(0,fn) or 'timer',destroy_timer=lambda t:timer.__setitem__(0,None),destroy_publisher=lambda p:destroyed.append(p))
  def pump(dt):
   if timer[0]:yaw[0]+=.01;timer[0]()
  rt=SimpleNamespace(node=node,ownership=lambda *a:None,pump=pump,record=lambda row:None)
  s=SimpleNamespace(rt=rt,options={},check=lambda c:None)
  sweep=ContinuousSweep(s);sweep.heading=lambda:yaw[0]
  def bad_capture():raise ValueError('invalid camera')
  sweep.acquire=bad_capture
  with patch.dict(sys.modules,{'geometry_msgs':SimpleNamespace(),'geometry_msgs.msg':SimpleNamespace(Twist=Twist)}):
   with self.assertRaisesRegex(ValueError,'invalid camera'):sweep.run('bear_doll',Event(),'test')
  self.assertIn(.4,sent);self.assertEqual(sent[-3:],[0.,0.,0.]);self.assertEqual(destroyed,[pub]);self.assertIsNone(timer[0])

class PoseRecoveryTests(unittest.TestCase):
 def make(self,heading,check=lambda c:None):
  from continuous_scan import ContinuousSweep
  self.zeros=[];self.records=[];self.pumps=[]
  rt=SimpleNamespace(record=self.records.append,pump=self.pumps.append)
  sweep=ContinuousSweep(SimpleNamespace(rt=rt,options={},check=check))
  sweep.heading=heading
  return sweep
 def test_stale_and_future_pose_wait_then_recover(self):
  seq=iter([RuntimeError('continuous_scan_stale_pose'),RuntimeError('continuous_scan_stale_pose'),1.2])
  def heading():
   value=next(seq)
   if isinstance(value,Exception):raise value
   return value
  sweep=self.make(heading)
  self.assertEqual(sweep.wait_heading(Event(),lambda:self.zeros.append(0)),1.2)
  self.assertEqual(self.zeros,[0,0]);self.assertEqual(len(self.pumps),2)
  self.assertEqual([r['event'] for r in self.records],['continuous_scan_pose_wait','continuous_scan_pose_recovered'])
 def test_wait_remains_cancellable(self):
  cancel=Event()
  def check(c):
   if c.is_set():raise RuntimeError('task_cancelled')
  def stale():raise RuntimeError('continuous_scan_stale_pose')
  sweep=self.make(stale,check);sweep.rt.pump=lambda dt:cancel.set()
  with self.assertRaisesRegex(RuntimeError,'task_cancelled'):
   sweep.wait_heading(cancel,lambda:self.zeros.append(0))
  self.assertEqual(self.zeros,[0])
 def test_other_fault_is_not_swallowed(self):
  def bad():raise RuntimeError('ownership_lost')
  sweep=self.make(bad)
  with self.assertRaisesRegex(RuntimeError,'ownership_lost'):sweep.wait_heading(Event(),lambda:None)
