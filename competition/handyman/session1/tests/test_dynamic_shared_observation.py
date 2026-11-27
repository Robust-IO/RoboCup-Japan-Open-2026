import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from dynamic_bindings import DynamicBindings
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure
class SharedObservationTests(unittest.TestCase):
 def binding(self):
  b=DynamicBindings.__new__(DynamicBindings)
  b.task={'target':'dog_doll'}
  b.backend=SimpleNamespace(scene=object(),source_snapshot=object(),record=Mock())
  b.stages=SimpleNamespace(source_id='sofa',check=Mock(),navigate_source=Mock(),observe_target=Mock(return_value={'fresh':True}))
  b.set_source=Mock()
  return b
 def test_confirmation_uses_fixed_cycle_and_hands_off_once(self):
  b=self.binding();result={'confirmed':True,'target':'dog_doll','source_table':'sofa'}
  with patch('furniture_reobservation._observe_cycle',return_value=result) as cycle:
   self.assertIs(b.confirm_furniture('sofa',{},None),result)
   cycle.assert_called_once_with(b.stages,{},None)
  self.assertIs(b.observe_target({},None),result)
  b.stages.observe_target.assert_not_called()
  self.assertEqual(b.observe_target({},None),{'fresh':True})
 def test_changed_target_never_reuses_observation(self):
  b=self.binding();b._confirmed_source_observation=('dog_doll','sofa',{'confirmed':True})
  b.task['target']='canned_juice'
  self.assertEqual(b.observe_target({},None),{'fresh':True})
 def test_room_search_result_without_scene_uses_fixed_observation(self):
  b=self.binding();b._confirmed_source_observation=None
  self.assertEqual(b.observe_target({},None),{'fresh':True})
 def test_furniture_miss_returns_to_dynamic_search(self):
  b=self.binding()
  with patch('furniture_reobservation._observe_cycle',side_effect=AttemptFailure(Failure.OCCLUDED,'missing')):
   self.assertIsNone(b.confirm_furniture('sofa',{},None))
 def test_other_failures_are_not_reported_as_missing(self):
  b=self.binding()
  with patch('furniture_reobservation._observe_cycle',side_effect=ValueError('bad_feedback')):
   with self.assertRaisesRegex(ValueError,'bad_feedback'):b.confirm_furniture('sofa',{},None)
