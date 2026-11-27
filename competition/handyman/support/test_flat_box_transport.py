import unittest,cv2,numpy as np
from flat_box_transport import extents,silhouette
class BoxTransportTest(unittest.TestCase):
 def test_flat_extents_use_thickness_not_long_edge(self):
  v,h=extents(np.eye(3),[.084,.05,.0225]);self.assertAlmostEqual(v,.01125);self.assertAlmostEqual(h,np.hypot(.042,.025))
 def test_rotated_vertical_extent(self):
  r=np.array([[0,0,1],[0,1,0],[-1,0,0]]);v,_=extents(r,[.084,.05,.0225]);self.assertAlmostEqual(v,.042)
 def scene(self):
  hsv=np.zeros((480,640,3),np.uint8);hsv[30:430,:200]=[99,180,180];hsv[150:180,520:560]=[99,150,150]
  return cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
 def test_small_same_colour_background_not_held_object(self):
  image=self.scene();current=image.copy();current[320:340,400:420]=image[40:60,40:60]
  self.assertEqual(silhouette(image,current,dict(hue=99,tolerance=8))['iou'],1.)
 def test_missing_held_box_not_passed_by_background(self):
  image=self.scene();current=image.copy();current[30:430,:200]=0
  with self.assertRaises(ValueError):silhouette(image,current,dict(hue=99,tolerance=8))
if __name__=='__main__':unittest.main()
