import math,unittest
import numpy as np
import cv2
from horizontal_can_hand import project,cylinder_points,fit,follows_lift,hand_feedback_fresh

class HandCylinderTest(unittest.TestCase):
    def scene(self,center,height):
        s=dict(base_from_hand=dict(translation=[.49,.05,height],quaternion=[0.,1.,0.,0.]),
            hand_k=[205.47,0.,320.,0.,205.47,240.,0.,0.,1.],hand_d=[0.]*5,
            streams=dict(hand=dict(frame='hand_camera_rgb_frame',stamp_ns=123)))
        m=dict(center_base_m=[.52,.08,.534],axis_base=[0.,1.,0.],radius_m=.0377,
            observed_length_m=.122,coloured_length_m=.112)
        image=np.zeros((480,640,3),np.uint8)
        pixels=project(s,cylinder_points(center,[0,1,0],.0377,.112))
        hull=cv2.convexHull(pixels.astype(np.float32)).astype(np.int32)
        cv2.fillConvexPoly(image,hull,(0,210,100))
        return image,s,m,dict(hue=46,tolerance=19)

    def test_supported_pose_with_raster_rotation(self):
        center=[.55,.09,.534];image,s,m,a=self.scene(center,.768)
        v=fit(image,s,m,a,height=.534)
        np.testing.assert_allclose(v['center_m'],center,atol=.002)
        self.assertFalse(v['depth_measured'])

    def test_free_known_radius_distinguishes_lift_from_table(self):
        for z in (.534,.614):
            with self.subTest(z=z):
                center=[.50,.08,z];image,s,m,a=self.scene(center,.88)
                v=fit(image,s,m,a)
                np.testing.assert_allclose(v['center_m'],center,atol=.003)
                self.assertEqual(v['height_constraint'],'none')

    def test_missing_target_not_replaced_with_prior(self):
        image,s,m,a=self.scene([.55,.09,.534],.768);image[:]=0
        with self.assertRaises(ValueError):fit(image,s,m,a,height=.534)

    def texture(self):
        rng=np.random.default_rng(41)
        hsv=np.zeros((480,640,3),np.uint8)
        hsv[160:320,170:470,0]=46;hsv[160:320,170:470,1]=200
        values=cv2.resize(rng.integers(45,245,(40,75),dtype=np.uint8),(300,160),interpolation=cv2.INTER_NEAREST)
        hsv[160:320,170:470,2]=values
        return cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR),dict(hue=46,tolerance=19)

    def test_same_can_texture_stays_with_hand(self):
        before,a=self.texture()
        after=cv2.warpAffine(before,np.array([[1.,0,2.],[0,1.,-1.]]),(640,480))
        proof=follows_lift(before,after,a)
        self.assertGreater(proof['inliers'],8)
        self.assertAlmostEqual(proof['scale'],1.,delta=.01)

    def test_table_object_receding_after_lift_is_not_held(self):
        before,a=self.texture()
        after=cv2.warpAffine(before,cv2.getRotationMatrix2D((320,240),0,.65),(640,480))
        with self.assertRaisesRegex(ValueError,'does_not_follow'):
            follows_lift(before,after,a)

    def test_head_age_does_not_block_hand_only_controller(self):
        names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        snapshot=dict(tracking_epoch='trial',streams=dict(hand=dict(stamp_ns=800_000_000),rgb=dict(stamp_ns=1)),joints=dict.fromkeys(names,0.))
        state=dict(stamp_ns=950_000_000,joints=[0.]*6)
        proof=hand_feedback_fresh(snapshot,state,1_000_000_000,'trial')
        self.assertTrue(proof['head_image_not_used'])
        self.assertEqual(proof['original_image_stamp_ns'],800_000_000)

    def test_old_hand_image_still_stops_controller(self):
        snapshot=dict(tracking_epoch='trial',streams=dict(hand=dict(stamp_ns=500_000_000)))
        with self.assertRaisesRegex(ValueError,'fresh_hand_image_required'):
            hand_feedback_fresh(snapshot,{},1_000_000_000,'trial')

if __name__=='__main__':unittest.main()
