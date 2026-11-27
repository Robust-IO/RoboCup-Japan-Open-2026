import copy,unittest
import numpy as np
from pepper_geometry import source_support_plane,estimate_source
from test_pepper_geometry import scene

class SourceSupportTests(unittest.TestCase):
    def setUp(self):
        self.rgb,self.depth,self.s,self.d,self.body,self.cap=scene()
        self.s['tracking_transforms']=dict(map_from_base_at_depth=dict(stamp_ns=1_000_000_000,
            translation=[1.,2.,.1],quaternion=[0.,0.,0.,1.]))
        self.support=dict(plane_map=[0.,0.,.54],plane_points=500,residual_p95_m=0.,source_depth_stamp_ns=1_000_000_000)
    def test_measured_map_plane_transforms_to_observation_frame(self):
        np.testing.assert_allclose(source_support_plane(self.support,self.s),[0.,0.,.44],atol=1e-12)
        m=estimate_source(self.rgb,self.depth,self.s,self.d,measured_support=self.support)
        np.testing.assert_allclose(m['center_m'],self.body,atol=.0015)
        self.assertFalse(m['motion_authorized'])
    def test_old_noisy_or_unobserved_plane_is_rejected(self):
        for key,value in (('source_depth_stamp_ns',999),('residual_p95_m',.005),('plane_points',149)):
            changed=dict(self.support,**{key:value})
            with self.subTest(key=key),self.assertRaises(ValueError):source_support_plane(changed,self.s)
        bad=dict(self.support,plane_map=[0.,0.,.64])
        with self.assertRaisesRegex(ValueError,'not_observed_near_target'):
            estimate_source(self.rgb,self.depth,self.s,self.d,measured_support=bad)
    def test_measured_support_does_not_substitute_missing_object_depth(self):
        with self.assertRaises(ValueError):estimate_source(self.rgb,self.depth*0,self.s,self.d,measured_support=self.support)

if __name__=='__main__':unittest.main()
