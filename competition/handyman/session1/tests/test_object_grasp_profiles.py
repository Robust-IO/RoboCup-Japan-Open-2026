import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from object_grasp_profiles import apply,profile_for,FLAGS,catalog
from grasp_runtime.autograsp.contact import contact_fit
class ObjectProfileTests(unittest.TestCase):
 def test_fixed_targets_bind_to_class_not_round(self):
  mapping=catalog()['session_target_bindings']
  self.assertEqual(mapping['1'],mapping['3'])
  for n,target in mapping.items():
   b=SimpleNamespace(record=Mock());apply(b,target)
   self.assertEqual(b.skip_actual_pad_overlap,target=='filled_ketchup')
 def test_switching_target_clears_all_ketchup_overrides(self):
  b=SimpleNamespace(record=Mock());apply(b,'filled_ketchup')
  self.assertTrue(all(getattr(b,f) for f in FLAGS))
  for target in ('dog_doll','canned_juice','tumbler','rabbit_doll','apple'):
   apply(b,target);self.assertTrue(all(not getattr(b,f) for f in FLAGS))
   apply(b,'filled_ketchup')
 def fixture(self):
  contacts=np.array([[[x,y,z] for x in (0,.04) for z in (0,.04)] for y in (-.05,.05)])
  hand=SimpleNamespace(contacts=contacts)
  policy=SimpleNamespace(side_margin_m=.004,min_contact_overlap_m=.003,template_family='generic',table_margin_m=.005)
  points=np.array([[.02,y,z] for y in np.linspace(-.03,.03,15) for z in (.01,.02,.03)])
  return points,hand,policy
 def test_pad_overlap_override_does_not_fake_default_pass(self):
  points,hand,policy=self.fixture()
  self.assertEqual(contact_fit(points,np.zeros(3),np.eye(3),[0,0,0],hand,policy)['reason'],'insufficient_actual_pad_overlap')
  with patch('grasp_runtime.autograsp.table_clearance.hand_table_clearance',return_value=.02):
   result=contact_fit(points,np.zeros(3),np.eye(3),[0,0,0],hand,policy,require_pad_overlap=False)
  self.assertTrue(result['fits']);self.assertFalse(result['pad_overlap_check_enabled'])
 def test_other_contact_failures_remain(self):
  points,hand,policy=self.fixture()
  result=contact_fit(points[:2],np.zeros(3),np.eye(3),[0,0,0],hand,policy,require_pad_overlap=False)
  self.assertEqual(result['reason'],'contact_band_unobserved')
  with patch('grasp_runtime.autograsp.table_clearance.hand_table_clearance',return_value=-.01):
   result=contact_fit(points,np.zeros(3),np.eye(3),[0,0,0],hand,policy,require_pad_overlap=False)
  self.assertEqual(result['reason'],'actual_closing_hand_too_low')
