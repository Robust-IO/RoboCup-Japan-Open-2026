import unittest
from grasp_runtime.autograsp.strategy_retry import too_wide_for_side
class StrategyTests(unittest.TestCase):
 def test_narrow_cylinder_failure_is_not_width_evidence(self):
  self.assertFalse(too_wide_for_side({'observed_minimum_horizontal_width_m':.065,'side_candidate_width_checks':[{'local_span_width_m':.055}]}))
 def test_narrow_local_part_overrides_large_object_extent(self):
  self.assertFalse(too_wide_for_side({'observed_minimum_horizontal_width_m':.2,'side_candidate_width_checks':[{'local_span_width_m':.12},{'local_span_width_m':.04}]}))
 def test_only_overwide_local_spans_allow_switch(self):
  self.assertTrue(too_wide_for_side({'side_candidate_width_checks':[{'local_span_width_m':.11},{'local_span_width_m':.12}]}))
