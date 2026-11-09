import copy
import unittest
import numpy as np
from measure_destination_surface import measure as legacy
from can_destination_surface import measure
import test_measure_destination_surface as table_fixtures


class RobustTableTests(unittest.TestCase):
    def fixture(self):
        d,s,t=table_fixtures.TableDepthTest().fixture();s['streams']['rgb']=dict(stamp_ns=1)
        s['tracking_transforms']['map_from_base_at_depth'].update(source_frame='base_link',target_frame='map')
        return d,s,t
    def test_existing_clean_plane_result_is_unchanged(self):
        data=self.fixture();self.assertEqual(measure(*data),legacy(*data))
    def test_near_table_height_foreground_does_not_bias_plane(self):
        d,s,t=self.fixture();d[20:45,65:90]-=.0065
        with self.assertRaisesRegex(ValueError,'noisy_table_plane'):legacy(d,s,t)
        result=measure(d,s,t);self.assertAlmostEqual(result['table_height_map_m'],.5,places=6)
        self.assertGreater(result['near_height_outliers_removed'],100)
        self.assertFalse(result['residual_limit_relaxed']);self.assertLess(result['residual_p95_m'],.004)
    def test_unstructured_noise_is_not_invented_as_a_good_plane(self):
        d,s,t=self.fixture();d+=np.random.default_rng(31).normal(0,.01,d.shape)
        with self.assertRaises(ValueError):measure(d,s,t)
    def test_no_depth_or_wrong_transform_time_still_rejects(self):
        d,s,t=self.fixture();d*=0
        with self.assertRaises(ValueError):measure(d,s,t)
        d,s,t=self.fixture();s['tracking_transforms']['map_from_base_at_depth']['stamp_ns']=2
        with self.assertRaises(ValueError):measure(d,s,t)
    def test_raw_depth_and_snapshot_are_preserved(self):
        d,s,t=self.fixture();d[20:45,65:90]-=.0065;before=d.copy();snap=copy.deepcopy(s)
        measure(d,s,t);np.testing.assert_array_equal(d,before);self.assertEqual(s,snap)


if __name__=='__main__':unittest.main()
