import unittest
import numpy as np
from grasp_runtime.autograsp.tracking import register_surface,UnobservedSurface
class ExtentOverrideTests(unittest.TestCase):
    def test_small_real_patch_allowed_only_when_requested(self):
        x,y=np.meshgrid(np.linspace(-.1,.1,61),np.linspace(-.1,.1,61))
        template=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        observed=template[(abs(template[:,:2])<.02).all(1)]
        rgb=np.full_like(template,128);colour=np.full_like(observed,128)
        with self.assertRaisesRegex(UnobservedSurface,'only_tiny_surface_fragment_visible'):
            register_surface(template,rgb,observed,colour)
        result=register_surface(template,rgb,observed,colour,require_surface_extent=False)
        np.testing.assert_allclose(result.translation,0,atol=1e-8)
        self.assertGreaterEqual(result.distinct_template_points,30)
    def test_override_does_not_allow_missing_depth_points(self):
        points=np.zeros((5,3));rgb=np.zeros_like(points)
        with self.assertRaisesRegex(UnobservedSurface,'too_few_observed_surface_points'):
            register_surface(points,rgb,points,rgb,require_surface_extent=False)

    def test_optional_shift_limit(self):
        rng=np.random.default_rng(81)
        template=rng.uniform(-.1,.1,(5000,3))
        rgb=np.full_like(template,128)
        observed=template+np.array([.045,0,0])
        # Test explicit opt-out separately from default behaviour using a small
        # configured bound, so correspondence convergence is not assumed here.
        observed=template+np.array([.005,0,0])
        with self.assertRaisesRegex(UnobservedSurface,'surface_moved_beyond_registration_window'):
            register_surface(template,rgb,observed,rgb,max_shift=.001)
        result=register_surface(template,rgb,observed,rgb,max_shift=None)
        np.testing.assert_allclose(result.translation,[.005,0,0],atol=.001)
