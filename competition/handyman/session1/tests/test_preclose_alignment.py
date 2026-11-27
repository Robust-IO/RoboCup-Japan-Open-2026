import unittest
from types import SimpleNamespace
import numpy as np
from grasp_runtime.autograsp.preclose_alignment import align_side_before_close
class PrecloseTests(unittest.TestCase):
 def backend(self,errors):
  calls=[];b=SimpleNamespace(pose_locked=True,policy=SimpleNamespace(max_lift_m=.69,lift_check_m=.08))
  fits=iter([False,True]);b._actual_contact_check=lambda s:dict(fits=next(fits),reason='material_outside_actual_open_gap')
  b.preclose_height_delta=lambda error,s:float(error[2])
  b.rt=SimpleNamespace(contact_error=lambda *a:next(errors),state=lambda:{'joints':[.3,0,0,0,0,.65]})
  def observe():
   self.assertFalse(b.pose_locked);calls.append('camera');return {'snapshot':{}},{'center_m':[0,0,0],'depth_measured':True}
  b._observe_tracking=observe;b._tf=lambda s:(np.eye(3),np.zeros(3));b._cloud=lambda i:(np.zeros((3,3)),None,None)
  b.record=lambda e:None;b._arm=lambda g,l:calls.append(('arm',g));b._base=lambda d,l:calls.append(('base',d))
  return b,calls
 def test_locked_pose_is_remeasured_and_corrected_then_remeasured(self):
  b,c=self.backend(iter([[.012,-.018,0.],[0,0,0]]));align_side_before_close(b)
  self.assertEqual(c,['camera',('base',[.012,-.018,0.]),'camera']);self.assertTrue(b.pose_locked)
 def test_aligned_pose_does_not_add_movement(self):
  b,c=self.backend(iter([[.001,.001,0.]]));b._actual_contact_check=lambda s:dict(fits=True);align_side_before_close(b);self.assertEqual(c,['camera'])
 def test_camera_failure_never_falls_back_to_stale_pose(self):
  b,c=self.backend(iter([]))
  def fail():raise RuntimeError('occluded')
  b._observe_tracking=fail
  with self.assertRaises(RuntimeError):align_side_before_close(b)
  self.assertTrue(b.pose_locked);self.assertEqual(c,[])

 def test_floor_compensation_precedes_lateral_alignment(self):
  b,c=self.backend(iter([[.0027,-.0021,0.],[0,0,0]]))
  b.preclose_height_delta=lambda e,s:.001
  align_side_before_close(b)
  self.assertEqual(c[1][0],'arm');self.assertEqual(c[2][0],'base')

 def test_second_camera_residual_gets_another_alignment(self):
  b,c=self.backend(iter([[.013,.021,0.],[-.0106,.0098,0.],[0,0,0]]))
  fits=iter([False,False,True]);b._actual_contact_check=lambda s:dict(fits=next(fits),reason='material_outside_actual_open_gap')
  align_side_before_close(b)
  self.assertEqual(sum(isinstance(x,tuple) and x[0]=='base' for x in c),2)
