import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_turn_policy import same_search_position

def point(x=1.,y=2.,yaw=0.):
    return {'pose':dict(x=x,y=y,yaw=yaw)}

class SearchTurnPolicyTests(unittest.TestCase):
    def test_ring(self):
        self.assertTrue(same_search_position(point(),point(yaw=math.pi/4)))
    def test_wrap(self):
        self.assertTrue(same_search_position(point(yaw=3.1),point(yaw=3.1+math.pi/4-2*math.pi)))
    def test_backup_requires_navigation(self):
        self.assertFalse(same_search_position(point(),point(x=1.1,yaw=math.pi/4)))
    def test_bad_angles(self):
        for angle in (0.,-.5,2.,math.nan,math.inf):
            self.assertFalse(same_search_position(point(),point(yaw=angle)))
    def test_invalid(self):
        self.assertFalse(same_search_position({},point()))

if __name__=='__main__':unittest.main()
