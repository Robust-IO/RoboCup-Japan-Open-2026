import unittest
from grasp_runtime.autograsp.identity_policy import confidence_passes,within_source
class IdentityPolicyTests(unittest.TestCase):
    def test_strictly_above_half_for_explicit_source(self):
        self.assertTrue(confidence_passes(.56,True));self.assertFalse(confidence_passes(.5,True))
        self.assertFalse(confidence_passes(float('nan'),True))
    def test_unrestricted_threshold_unchanged(self):
        self.assertFalse(confidence_passes(.56));self.assertTrue(confidence_passes(.8))
    def test_wrong_furniture_is_rejected(self):
        t={'axis_aligned_bounds':{'center':{'x':1,'y':2},'size':{'x':.5,'y':.6}}}
        self.assertTrue(within_source([1,2,.5],t));self.assertFalse(within_source([2,2,.5],t))


class ComponentExceptionTests(unittest.TestCase):
    def points(self):
        import numpy as np
        rng=np.random.default_rng(21)
        return np.r_[rng.uniform(0,.008,(60,3)),rng.uniform(0,.008,(50,3))+[.06,0,0]]
    def test_above_85_accepts_both_measured_parts(self):
        from grasp_runtime.autograsp.identity_policy import identity_component
        mask,waived=identity_component(self.points(),.99)
        self.assertTrue(waived);self.assertEqual(mask.sum(),110)
    def test_disconnected_parts_do_not_reject_accepted_rgb_identity(self):
        from grasp_runtime.autograsp.identity_policy import identity_component
        for confidence in (.56,.786,.85):
            mask,disabled=identity_component(self.points(),confidence)
            self.assertTrue(disabled);self.assertEqual(mask.sum(),110)
    def test_high_confidence_does_not_invent_depth(self):
        import numpy as np
        from grasp_runtime.autograsp.identity_policy import identity_component
        from grasp_runtime.autograsp.tracking import UnobservedSurface
        with self.assertRaisesRegex(UnobservedSurface,'no_measured_depth_for_grasp_localization'):
            identity_component(np.empty((0,3)),.99)
    def test_single_component_unchanged(self):
        from grasp_runtime.autograsp.identity_policy import identity_component
        mask,waived=identity_component(self.points()[:60],.99)
        self.assertTrue(waived);self.assertEqual(mask.sum(),60)
