import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from preview_search_route import route


class Tests(unittest.TestCase):
    def test_shortest_four_connected(self):
        p=route(np.ones((4,4),bool),(0,0),(3,3))
        self.assertEqual(len(p),7)
        self.assertTrue(all(abs(a[0]-b[0])+abs(a[1]-b[1])==1 for a,b in zip(p,p[1:])))
    def test_blocked_goal(self):
        grid=np.ones((4,4),bool);grid[3,3]=False
        self.assertIsNone(route(grid,(0,0),(3,3)))
    def test_wall(self):
        grid=np.ones((4,4),bool);grid[2,:]=False
        self.assertIsNone(route(grid,(0,0),(3,3)))
    def test_no_diagonal_corner_cut(self):
        self.assertIsNone(route(np.eye(2,dtype=bool),(0,0),(1,1)))
    def test_invalid_start(self):self.assertIsNone(route(np.ones((2,2),bool),(-1,0),(1,1)))


if __name__=='__main__':unittest.main()
