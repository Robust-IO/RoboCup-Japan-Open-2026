import math,unittest
import numpy as np
from flat_box_geometry import box_from_points

class FlatBoxGeometryTest(unittest.TestCase):
    def scene(self,height=.012,yaw=.4):
        x,y=np.meshgrid(np.linspace(-.016,.016,30),np.linspace(-.030,.030,40))
        r=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        xy=np.c_[x.ravel(),y.ravel()]@r.T+[.8,.05]
        points=np.c_[xy,np.full(len(xy),.5+height)]
        x,y=np.meshgrid(np.linspace(.55,1.05,35),np.linspace(-.2,.3,35))
        support=np.c_[x.ravel(),y.ravel(),np.full(x.size,.5)]
        return points,support

    def test_flat_dimensions_not_cube_or_cylinder(self):
        for yaw in (0.,.4,1.2):
            with self.subTest(yaw=yaw):
                m=box_from_points(*self.scene(yaw=yaw))
                np.testing.assert_allclose(m['center_m'],[.8,.05,.506],atol=1e-6)
                self.assertAlmostEqual(m['length_m'],.06,places=5)
                self.assertAlmostEqual(m['width_m'],.032,places=5)
                self.assertAlmostEqual(m['height_m'],.012,places=5)

    def test_fingers_can_overlap_thin_side_without_going_below_table(self):
        m=box_from_points(*self.scene())
        self.assertAlmostEqual(m['finger_lower_clearance_m'],.002)
        self.assertAlmostEqual(m['estimated_finger_side_overlap_m'],.010)
        self.assertGreater(m['grasp_point_base_m'][2],m['center_m'][2])

    def test_table_plane_is_not_a_thin_box(self):
        with self.assertRaises(ValueError):box_from_points(*self.scene(height=0.))

    def test_missing_depth_not_replaced_by_dimensions(self):
        p,s=self.scene();p[:]=np.nan
        with self.assertRaises(ValueError):box_from_points(p,s)

if __name__=='__main__':unittest.main()
