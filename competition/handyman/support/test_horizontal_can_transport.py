import math,unittest
import numpy as np
import cv2
from horizontal_can_grasp_plan import forward
from horizontal_can_transport_geometry import cylinder_extents,held_silhouette
import test_horizontal_can_hand as hand_fixtures

class CylinderCarryTest(unittest.TestCase):
    def test_top_down_fold_turns_can_upright(self):
        _,r=forward([.4996,-math.pi/2,0,-math.pi/2,1.5168,.4778])
        local=r.T@np.array([-.054,.9985,0.])
        _,folded=forward([0,0,0,-math.pi/2,0,.4778])
        self.assertGreater(abs((folded@local)[2]),.999)

    def test_vertical_extent_is_half_length_not_radius(self):
        vertical,horizontal=cylinder_extents([0,0,1],.0377,.122)
        self.assertAlmostEqual(vertical,.061)
        self.assertAlmostEqual(horizontal,.0377)

    def test_already_held_can_does_not_require_texture_features(self):
        image,_,_,appearance=hand_fixtures.HandCylinderTest().scene([.55,.09,.534],.768)
        proof=held_silhouette(image,image.copy(),appearance)
        self.assertEqual(proof['iou'],1.)

    def test_receding_table_can_rejected(self):
        image,_,_,appearance=hand_fixtures.HandCylinderTest().scene([.55,.09,.534],.768)
        after=cv2.warpAffine(image,cv2.getRotationMatrix2D((320,240),0,.65),(640,480))
        with self.assertRaises(ValueError):held_silhouette(image,after,appearance)

    def test_unrelated_green_background_does_not_hide_real_held_can(self):
        image,_,_,appearance=hand_fixtures.HandCylinderTest().scene([.55,.09,.534],.768)
        after=image.copy();after[10:40,580:620]=[0,210,100]
        self.assertEqual(held_silhouette(image,after,appearance)['iou'],1.)

if __name__=='__main__':unittest.main()
