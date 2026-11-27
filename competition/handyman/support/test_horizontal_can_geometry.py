import math
import unittest
import numpy as np
from horizontal_can_geometry import horizontal_cylinder_from_points


class HorizontalCanGeometryTest(unittest.TestCase):
    def scene(self, yaw=0., noise=.0002):
        axis = np.array([math.cos(yaw), math.sin(yaw), 0.])
        across = np.array([-math.sin(yaw), math.cos(yaw), 0.])
        along, arc = np.meshgrid(np.linspace(-.0625,.0625,55), np.linspace(.15,math.pi-.15,35))
        center = np.array([.9,-.08,.7375])
        points = center + along.ravel()[:,None]*axis + (.0375*np.cos(arc.ravel()))[:,None]*across
        points[:,2] += .0375*np.sin(arc.ravel())
        points += np.random.default_rng(7).normal(0,noise,points.shape)
        x,y = np.meshgrid(np.linspace(.6,1.2,40),np.linspace(-.4,.25,40))
        support = np.c_[x.ravel(),y.ravel(),np.full(x.size,.7)]
        return points,support,center,axis

    def test_rotations_keep_measured_axis_and_dimensions(self):
        for yaw in (0.,.4,math.pi/2,2.3):
            with self.subTest(yaw=yaw):
                p,s,center,axis=self.scene(yaw)
                m=horizontal_cylinder_from_points(p,s)
                np.testing.assert_allclose(m['center_base_m'],center,atol=.002)
                self.assertGreater(abs(np.dot(m['axis_base'],axis)),.998)
                self.assertAlmostEqual(m['diameter_m'],.075,delta=.002)
                self.assertAlmostEqual(m['observed_length_m'],.125,delta=.005)
                self.assertFalse(m['motion_authorized'])

    def test_suspended_cylinder_not_assumed_resting(self):
        p,s,_,_=self.scene();p[:,2]+=.025
        with self.assertRaises(ValueError):horizontal_cylinder_from_points(p,s)

    def test_plane_is_not_a_cylinder(self):
        p,s,_,_=self.scene();p[:,2]=.76
        with self.assertRaises(ValueError):horizontal_cylinder_from_points(p,s)

    def test_zero_and_nan_depth_not_reconstructed(self):
        for p in (np.zeros((300,3)),np.full((300,3),np.nan),np.zeros((20,3))):
            with self.subTest(shape=p.shape):
                with self.assertRaises(ValueError):horizontal_cylinder_from_points(p,self.scene()[1])

    def test_missing_table_rejects(self):
        with self.assertRaises(ValueError):horizontal_cylinder_from_points(self.scene()[0],np.zeros((10,3)))

if __name__=='__main__':unittest.main()
