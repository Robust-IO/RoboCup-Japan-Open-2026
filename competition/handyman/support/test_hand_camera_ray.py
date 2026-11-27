import unittest
import numpy as np
from hand_camera_ray import rays


class RayTests(unittest.TestCase):
    def setUp(self):
        self.snapshot = dict(streams={'hand': dict(width=640,height=480,stamp_ns=1,frame='hand_camera_rgb_frame')},
                             hand_d=[0]*5,hand_k=[200,0,320,0,200,240,0,0,1],
                             base_from_hand=dict(translation=[0,0,0],quaternion=[0,0,0,1]),
                             palm_from_hand=dict(translation=[0,0,0],quaternion=[0,0,0,1]))

    def test_center(self):
        r=rays(self.snapshot,[319,239,321,241],'sigverse_hsr_hand_v2')
        np.testing.assert_allclose(r['base_ray']['unit_direction'],[0,0,1])
        self.assertFalse(r['motion_authorized'])

    def test_raw_axes(self):
        r=rays(self.snapshot,[119,239,121,241],'sigverse_hsr_hand_v2')
        np.testing.assert_allclose(r['base_ray']['unit_direction'],np.array([0,1,1])/np.sqrt(2))
        r=rays(self.snapshot,[319,39,321,41],'sigverse_hsr_hand_v2')
        np.testing.assert_allclose(r['base_ray']['unit_direction'],np.array([-1,0,1])/np.sqrt(2))

    def test_old_profile_rejected(self):
        with self.assertRaises(ValueError):rays(self.snapshot,[319,239,321,241],'sigverse_hsr_raw_hand')

    def test_v2_rotation_is_proper(self):
        from hand_camera_ray import HAND_RASTER_TO_FRAME
        np.testing.assert_allclose(HAND_RASTER_TO_FRAME.T@HAND_RASTER_TO_FRAME,np.eye(3))
        self.assertAlmostEqual(np.linalg.det(HAND_RASTER_TO_FRAME),1.)

    def test_reject_bad_box(self):
        with self.assertRaises(ValueError):rays(self.snapshot,[-1,0,2,2])

    def test_reject_distortion(self):
        self.snapshot['hand_d'][0]=.1
        with self.assertRaises(ValueError):rays(self.snapshot,[1,1,2,2])

    def test_reject_wrong_frame(self):
        self.snapshot['streams']['hand']['frame']='other'
        with self.assertRaises(ValueError):rays(self.snapshot,[1,1,2,2],'sigverse_hsr_hand_v2')


if __name__=='__main__':unittest.main()
