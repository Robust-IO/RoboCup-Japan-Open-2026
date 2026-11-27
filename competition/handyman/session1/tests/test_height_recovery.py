import unittest
from grasp_runtime.autograsp.height_recovery import recovery_raise
from grasp_runtime.autograsp.coordinator import AttemptFailure
from grasp_runtime.autograsp.recovery import Failure
class HeightRecoveryTests(unittest.TestCase):
 def error(self,clearance,reason='open_hand_path_too_low_for_table'):
  e=AttemptFailure(Failure.APPROACH_COLLISION,reason);e.clearance_m=clearance;e.required_m=.005;return e
 def test_small_shortfall_has_positive_buffer(self):self.assertAlmostEqual(recovery_raise(self.error(.004)),.003)
 def test_large_shortfall_is_not_blindly_lifted(self):self.assertIsNone(recovery_raise(self.error(-.04)))
 def test_obstacle_is_not_table_recovery(self):self.assertIsNone(recovery_raise(self.error(.004,'open_hand_path_intersects_observed_surface')))
 def test_missing_measurement_not_bypassed(self):self.assertIsNone(recovery_raise(RuntimeError('open_hand_path_too_low_for_table')))
