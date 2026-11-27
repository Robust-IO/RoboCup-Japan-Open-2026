import unittest
import numpy as np
from container_rim_calibration import calibrate
class RimTests(unittest.TestCase):
 def opening(self):return dict(polygon_map_m=[[-.15,-.15],[.15,-.15],[.15,.15],[-.15,.15]],rim_height_m=.58)
 def cloud(self):
  t=np.linspace(-.15,.15,61)
  xy=np.vstack([np.c_[t,t*0-.15],np.c_[t,t*0+.15],np.c_[t*0-.15,t],np.c_[t*0+.15,t]])
  return np.c_[xy+[.07,-.05],np.full(len(xy),.58)]
 def test_translation_recovered(self):
  updated,proof=calibrate(self.cloud(),self.opening())
  np.testing.assert_allclose(proof['translation_xy_m'],[.07,-.05],atol=.003)
 def test_single_edge_not_a_mouth(self):
  with self.assertRaises(ValueError):calibrate(self.cloud()[:61],self.opening())
 def test_no_current_rim_rejected(self):
  p=self.cloud();p[:,2]=0
  with self.assertRaises(ValueError):calibrate(p,self.opening())
