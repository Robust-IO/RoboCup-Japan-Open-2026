import unittest,math
import numpy as np
from object_support_disk import enclosing_disk,disk_inside
from release_geometry import release_check

class SupportDiskTests(unittest.TestCase):
    def bounds(self):return dict(center=dict(x=0,y=0),size=dict(x=1,y=1))
    def test_tight_triangle_circle_contains_all_points(self):
        p=np.array([[0,0],[2,0],[1,math.sqrt(3)]])
        c,r=enclosing_disk(p)
        np.testing.assert_allclose(c,[1,math.sqrt(3)/3],atol=1e-8)
        self.assertAlmostEqual(r,2/math.sqrt(3))
        self.assertTrue((np.linalg.norm(p-c,axis=1)<=r+1e-9).all())
    def test_line_and_duplicate_points(self):
        c,r=enclosing_disk([[0,0],[.2,0],[.2,0],[.1,0]])
        np.testing.assert_allclose(c,[.1,0]);self.assertAlmostEqual(r,.1)
    def test_entire_circle_not_just_center_must_fit(self):
        self.assertFalse(disk_inside([.45,0],.1,self.bounds())['safe'])
        self.assertTrue(disk_inside([.39,0],.1,self.bounds())['safe'])
    def test_release_ignores_disk_but_requires_gripper_inside(self):
        table=dict(id='t',axis_aligned_bounds=self.bounds());plane=dict(destination_id='t',plane_map=[0,0,.5])
        plan=dict(destination_id='t',radius_m=.1,placement_selection_basis='tight_object_disk')
        self.assertFalse(release_check([.55,0,.6],[[-.1,0,.52],[.1,0,.52]],plane,table,plan,(0,.05))['ready'])
        self.assertTrue(release_check([.4,0,.6],[[.35,0,.52],[.55,0,.52]],plane,table,plan,(0,.05))['ready'])
        self.assertTrue(release_check([0,0,.6],[[-.1,0,.52],[.1,0,.52]],plane,table,plan,(0,.05))['ready'])
    def test_random_points_all_enclosed(self):
        p=np.random.default_rng(5).normal(size=(400,2))*.1;c,r=enclosing_disk(p)
        self.assertTrue((np.linalg.norm(p-c,axis=1)<=r+1e-9).all())
