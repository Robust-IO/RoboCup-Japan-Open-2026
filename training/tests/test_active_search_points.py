import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from build_active_search_points import disk_free, distances, select_points


class SearchPointTests(unittest.TestCase):
    def test_outside_map_is_blocked(self):
        a=disk_free(np.ones((20,20),bool),.1,.25)
        self.assertFalse(a[0,0]);self.assertTrue(a[10,10])

    def test_obstacle_blocks_rotation(self):
        a=np.ones((20,20),bool);a[10,10]=False
        allowed=disk_free(a,.1,.25)
        self.assertFalse(allowed[10,12]);self.assertTrue(allowed[10,15])

    def test_diagonal_corner_not_route(self):
        d=distances(np.eye(2,dtype=bool),(0,0))
        self.assertEqual(d[1,1],-1)

    def test_center_obstacle_selects_other_position(self):
        a=np.ones((30,30),bool);a[13:17,13:17]=False
        r=select_points(a,.1,[0,0,0],(.5,.5),{'r':{'region':[[0,0],[3,0],[3,3],[0,3]]}},.15)[0]
        self.assertEqual(r['status'],'proposed');self.assertEqual(len(r['views']),8)
        self.assertFalse(1.3<r['main']['x']<1.7 and 1.3<r['main']['y']<1.7)
        self.assertFalse(r['actionable'])

    def test_disconnected_room_not_absence(self):
        a=np.ones((30,30),bool);a[:,15]=False
        r=select_points(a,.1,[0,0,0],(.5,.5),{'r':{'region':[[2,0],[3,0],[3,3],[2,3]]}},.1)[0]
        self.assertEqual(r['status'],'no_conservative_candidate')
        self.assertFalse(r['does_not_exist_authorized'])

    def test_invalid_start_no_teleport(self):
        r=select_points(np.ones((10,10),bool),.1,[0,0,0],(-1,-1),
                        {'r':{'region':[[0,0],[1,0],[1,1],[0,1]]}},.1)[0]
        self.assertEqual(r['status'],'start_requires_review');self.assertIsNone(r['main'])

    def test_rotated_map_coordinates(self):
        import math
        r=select_points(np.ones((20,20),bool),.1,[5,5,math.pi/2],(4.5,5.5),
                        {'r':{'region':[[3,5],[5,5],[5,7],[3,7]]}},.1)[0]
        self.assertEqual(r['status'],'proposed');self.assertTrue(3<r['main']['x']<5)

    def test_invalid_resolution(self):
        with self.assertRaises(ValueError):disk_free(np.ones((3,3),bool),0,.1)


if __name__=='__main__':unittest.main()
