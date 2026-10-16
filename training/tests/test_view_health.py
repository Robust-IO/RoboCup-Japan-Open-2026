import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from view_health import assess_view

class ViewHealthTests(unittest.TestCase):
    def rgb(self):
        rgb=np.zeros((10,10,3),np.uint8);rgb[5:]=255;return rgb
    def test_good_not_coverage(self):
        r=assess_view(self.rgb(),np.ones((10,10)))
        self.assertTrue(r['data_usable']);self.assertFalse(r['coverage_verified'])
    def test_zero_depth(self): self.assertFalse(assess_view(self.rgb(),np.zeros((10,10)))['data_usable'])
    def test_nonfinite_depth(self): self.assertFalse(assess_view(self.rgb(),np.full((10,10),np.nan))['data_usable'])
    def test_flat_colour(self):
        rgb=np.zeros((10,10,3),np.uint8);rgb[:,:,0]=255
        self.assertFalse(assess_view(rgb,np.ones((10,10)))['data_usable'])
    def test_wrong_shape(self): self.assertFalse(assess_view(self.rgb(),np.ones((3,3)))['data_usable'])
    def test_range(self):
        for z in [.4,8.,9.]: self.assertFalse(assess_view(self.rgb(),np.full((10,10),z))['data_usable'])

if __name__=='__main__': unittest.main()
