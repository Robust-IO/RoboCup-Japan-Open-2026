import unittest
from unittest.mock import patch
import numpy as np
from rgbd_pixel_window import pixel_window
from test_apple_inference_reuse import sphere_fixture
from apple_rgbd_candidate import estimate


class WindowTests(unittest.TestCase):
    def test_exact_original_full_frame_predicate_including_fractional_and_clipped_edges(self):
        v,u=np.indices((17,23))
        for bounds in ((2.2,3.8,14.1,15.),(-25.,-3.5,5.9,5.01),(20.,15.,50.,40.),
                       (0.,0.,23.,17.),(24.,0.,30.,17.),(3.,18.,8.,20.)):
            left,top,right,bottom=bounds
            expected=(u>=left)&(u<right)&(v>=top)&(v<bottom)
            region,local_u,local_v=pixel_window((17,23),*bounds)
            np.testing.assert_array_equal(local_u.ravel(),u[expected])
            np.testing.assert_array_equal(local_v.ravel(),v[expected])
            self.assertEqual(local_u.shape,np.zeros((17,23))[region].shape)

    def test_far_background_cannot_change_current_fit(self):
        fixture=sphere_fixture()
        expected=estimate(fixture.rgb,fixture.depth,fixture.s,fixture.d)
        rgb=fixture.rgb.copy();depth=fixture.depth.copy()
        rgb[:20]=[255,0,0];depth[:20]=.25
        self.assertEqual(expected,estimate(rgb,depth,fixture.s,fixture.d))

    def test_fit_only_allocates_local_pixel_grid(self):
        fixture=sphere_fixture()
        original=np.indices
        with patch('rgbd_pixel_window.np.indices',wraps=original) as grid:
            estimate(fixture.rgb,fixture.depth,fixture.s,fixture.d)
        self.assertEqual(grid.call_args.args[0],(30,30))

    def test_invalid_bounds_rejected(self):
        for bad in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):pixel_window((17,23),0.,0.,bad,10.)


if __name__=='__main__':unittest.main()
