import copy
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from verify_live_map import compare


class MapTests(unittest.TestCase):
    def setUp(self):
        self.map=S(header=S(frame_id='map'),info=S(width=2,height=2,resolution=.05,
            origin=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),data=[0,100,-1,0])
    def test_same(self):self.assertTrue(compare(self.map,self.map)['matches'])
    def test_cell(self):
        m=copy.deepcopy(self.map);m.data[0]=100
        self.assertEqual(compare(self.map,m)['reason'],'map_cells_mismatch')
    def test_origin(self):
        m=copy.deepcopy(self.map);m.info.origin.position.x=.05
        self.assertFalse(compare(self.map,m)['matches'])
    def test_resolution(self):
        m=copy.deepcopy(self.map);m.info.resolution=.1
        self.assertFalse(compare(self.map,m)['matches'])
    def test_dimensions(self):
        m=copy.deepcopy(self.map);m.info.width=1
        self.assertFalse(compare(self.map,m)['matches'])
    def test_frame(self):
        m=copy.deepcopy(self.map);m.header.frame_id='odom'
        self.assertFalse(compare(self.map,m)['matches'])
    def test_nan(self):
        m=copy.deepcopy(self.map);m.info.origin.position.x=float('nan')
        self.assertFalse(compare(self.map,m)['matches'])
    def test_short_data(self):
        m=copy.deepcopy(self.map);m.data.pop()
        self.assertFalse(compare(self.map,m)['matches'])

if __name__=='__main__':unittest.main()
