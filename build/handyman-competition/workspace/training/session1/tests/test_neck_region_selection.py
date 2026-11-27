import unittest
import numpy as np
from grasp_runtime.autograsp.shape_contacts import contact_preference
class NeckRegionTests(unittest.TestCase):
 def cloud(self):
  p=[]
  for z in np.linspace(0,.24,100):
   radius=.035 if z<.16 else .012
   for a in np.linspace(0,2*np.pi,36,endpoint=False):p.append([radius*np.cos(a),radius*np.sin(a),z])
  return np.array(p)
 def test_body_cannot_be_labelled_neck(self):
  _,reason=contact_preference(self.cloud(),[0,0,.10],'neck_bottle');self.assertEqual(reason,'contact_outside_observed_neck')
 def test_narrow_neck_accepted(self):
  shape,reason=contact_preference(self.cloud(),[0,0,.20],'neck_bottle');self.assertIsNone(reason);self.assertLess(shape['neck_width_m'],.03)

 def stepped_cloud(self):
  p=[]
  for z in np.linspace(0,.24,160):
   radius=.035 if z<.14 else (.020 if z<.20 else .012)
   for a in np.linspace(0,2*np.pi,36,endpoint=False):p.append([radius*np.cos(a),radius*np.sin(a),z])
  return np.array(p)
 def test_wider_usable_neck_is_accepted(self):
  shape,reason=contact_preference(self.stepped_cloud(),[0,0,.17],'neck_bottle')
  self.assertIsNone(reason)
  self.assertGreater(shape['neck_width_m'],.03)
  self.assertLess(shape['neck_width_m'],.05)
 def test_body_still_rejected_with_multiple_neck_widths(self):
  _,reason=contact_preference(self.stepped_cloud(),[0,0,.08],'neck_bottle')
  self.assertEqual(reason,'contact_outside_observed_neck')
 def test_rotated_neck_region_is_accepted(self):
  a=.65;R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
  _,reason=contact_preference(self.stepped_cloud()@R.T,R@np.array([0,0,.17]),'neck_bottle')
  self.assertIsNone(reason)
