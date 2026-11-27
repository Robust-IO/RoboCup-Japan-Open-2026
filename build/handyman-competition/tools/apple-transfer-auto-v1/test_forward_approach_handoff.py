import unittest
import numpy as np
from apple_approach_lease import closure_position_error
from run_bedroom_apple_regrasp import forward_approach_delta


class HandoffTests(unittest.TestCase):
    def test_actual_r13c_endpoint_enters_close_checks_not_negative_forward_failure(self):
        target=[.7027987227410115,.07279410526853293,.538142458345037]
        height=.2771978151685961+.2525409131088736
        self.assertIsNone(forward_approach_delta(target,height))
        np.testing.assert_allclose(closure_position_error(target,height),[.0135035042410115,-.00520589473146707,.0084037300675673],atol=1e-12)

    def test_lateral_offset_within_original_close_region_needs_no_four_mm_coarse_correction(self):
        self.assertIsNone(forward_approach_delta([.704,.088,.536],.53))

    def test_forward_still_plans_same_original_step_outside_close_region(self):
        np.testing.assert_allclose(forward_approach_delta([.86433466549,-.00357661476,.536],.53),[.16033466549,-.08157661476])

    def test_negative_forward_outside_close_region_still_stops(self):
        with self.assertRaisesRegex(RuntimeError,'requires_reviewed_reposition'):
            forward_approach_delta([.66,.078,.536],.53)

    def test_close_region_not_widened_for_lateral_or_vertical_error(self):
        for target in ([.702,.091,.536],[.702,.078,.551]):
            with self.assertRaisesRegex(ValueError,'target_outside_reviewed_closure_region'):
                closure_position_error(target,.53)
        with self.assertRaises(RuntimeError):forward_approach_delta([.702,.091,.536],.53)
        # Legacy coarse-stage tolerance may finish, but final fresh closure
        # still rejects this height; None is deliberately NOT close authority.
        self.assertIsNone(forward_approach_delta([.702,.078,.551],.53))

    def test_large_forward_keeps_old_bound_and_never_adds_backward_correction(self):
        delta=forward_approach_delta([1.1,.078,.536],.53)
        self.assertAlmostEqual(float(np.linalg.norm(delta)),.22);self.assertGreater(delta[0],0)

    def test_invalid_target_and_height_never_produce_step(self):
        for target in ([.7,.078,float('nan')],[.7,.078],[float('inf'),.078,.53]):
            with self.assertRaises(ValueError):forward_approach_delta(target,.53)
        with self.assertRaises(RuntimeError):forward_approach_delta([.7,.078,.53],float('nan'))


if __name__=='__main__':unittest.main()
