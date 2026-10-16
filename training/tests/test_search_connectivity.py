import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_search_connectivity import flood,cell,world

class ConnectivityTests(unittest.TestCase):
    def test_wall(self):
        a=np.ones((5,5),dtype=bool); a[:,2]=False
        self.assertEqual(int(flood(a,(0,0)).sum()),10)
    def test_gap(self):
        a=np.ones((5,5),dtype=bool);a[:,2]=False;a[2,2]=True
        self.assertEqual(int(flood(a,(0,0)).sum()),21)
    def test_invalid_start(self): self.assertFalse(flood(np.ones((2,2),bool),(-1,0)).any())
    def test_rotated_origin_roundtrip(self):
        for x,y in [(0,0),(10,5)]: self.assertEqual(cell(*world(x,y,[1,-2,.5],.05),[1,-2,.5],.05),(x,y))

if __name__=='__main__': unittest.main()
