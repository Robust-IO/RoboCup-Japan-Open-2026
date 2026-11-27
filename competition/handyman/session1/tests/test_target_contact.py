import unittest
import itertools
import numpy as np
from grasp_runtime.autograsp.target_contact import permitted_target_contact
class TargetContactTests(unittest.TestCase):
 def test_only_target_at_pads_is_permitted(self):
  pad=np.array(list(itertools.product([-.01,.01],[.04,.05],[-.01,.01])))
  pads=np.array([pad,pad*[1,-1,1]])
  points=np.array([[0,.045,0],[0,0,0],[0,.1,0],[0,-.045,0]])
  target=points[:3]
  self.assertEqual(permitted_target_contact(points,target,np.zeros(3),np.eye(3),pads).tolist(),[True,False,False,False])
 def test_empty_target_does_not_waive_anything(self):
  self.assertFalse(permitted_target_contact(np.array([[0,0,0]]),np.empty((0,3)),np.zeros(3),np.eye(3),[]).any())
