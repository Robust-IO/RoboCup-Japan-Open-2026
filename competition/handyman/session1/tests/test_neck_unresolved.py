import unittest
import numpy as np
from grasp_runtime.autograsp.shape_contacts import contact_preference
from grasp_runtime.autograsp.neck_sections import sections
class NeckUnresolvedTests(unittest.TestCase):
 def cloud(self,neck):
  return np.array([[(.012 if neck and z>.17 else .035)*np.cos(a),(.012 if neck and z>.17 else .035)*np.sin(a),z] for z in np.linspace(0,.24,100) for a in np.linspace(0,2*np.pi,32,endpoint=False)])
 def test_uniform_body_never_silently_accepted(self):
  shape,reason=contact_preference(self.cloud(False),[0,0,.12],'neck_bottle');self.assertIsNone(shape);self.assertEqual(reason,'neck_requires_new_observation')
 def test_candidates_are_actual_measured_neck_centers(self):
  p=self.cloud(True);_,_,_,rows=sections(p);self.assertTrue(rows)
  for _,w,c in rows:
   self.assertGreater(c[2],.17);self.assertLess(w,.03)
   self.assertIsNone(contact_preference(p,c,'neck_bottle')[1])
