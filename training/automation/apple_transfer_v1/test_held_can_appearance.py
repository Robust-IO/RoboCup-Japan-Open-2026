import copy
import json
from pathlib import Path
import subprocess
import sys
import unittest

import cv2
import numpy as np

from hand_camera_ray import rotation
from held_can_appearance import assess,build_appearance,project_held_can
from test_upright_cylinder_geometry import rendered_cylinder


def template():
    rgb,depth,s,d,center=rendered_cylinder()
    v,u=np.indices(depth.shape);k=np.asarray(s['rgb_k']).reshape(3,3)
    xyz=np.stack(((u-k[0,2])*depth/k[0,0],(v-k[1,2])*depth/k[1,1],depth),axis=-1)
    xyz=xyz@rotation(s['base_from_rgb']['quaternion']).T+s['base_from_rgb']['translation']
    top=(np.linalg.norm(xyz[:,:,:2]-center[:2],axis=2)<.026)&(abs(xyz[:,:,2]-.55)<.003)
    rgb[top]=[190,190,190]
    measurement=dict(class_name='canned_juice',center_m=center.tolist(),footprint_radius_m=.025,height_m=.11,observed_top_z_m=.55)
    return rgb,depth,s,measurement


def current_scene(appearance):
    ref=dict(can_grasp_and_lift_verified=True,held_opening_rad=.48,
        held_center_minus_palm_base_m=[.06655129582440844,.0002322477522304911,.0006584104449369743],
        can_height_m=.12506792828065508,can_footprint_radius_m=.03767738938288809)
    names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
    s=dict(palm_base_at_depth=[.28660203283652663,.07800000160932541,.6764923576265574],
        joints=dict(zip(names,[0.,0.,0.,-np.pi/2,0.,.48])),
        streams=dict(rgb=dict(width=640,height=480)),rgb_k=[554,0,320,0,554,240,0,0,1],
        base_from_rgb=dict(translation=[.053131378122826284,.02199999988079071,.9790874165917528],
            quaternion=[.6081581921967667,-.6081581921967666,.3607542283382977,-.3607542283382977]))
    rgb=np.full((480,640,3),225,np.uint8)
    color=cv2.cvtColor(np.array([[[appearance['hue'],190,190]]],np.uint8),cv2.COLOR_HSV2RGB)[0,0].tolist()
    cv2.rectangle(rgb,(185,350),(297,430),color,-1)
    cv2.ellipse(rgb,(238,346),(55,40),-11,0,360,(185,185,185),-1)
    cv2.ellipse(rgb,(238,346),(55,40),-11,0,360,(55,55,55),2)
    cv2.ellipse(rgb,(238,346),(46,31),-11,0,360,(75,75,75),2)
    cv2.ellipse(rgb,(238,346),(12,18),0,0,360,(75,75,75),2)
    rgb[430:]=[245,245,245]
    depth=np.full((480,640),.8,np.float32);depth[280:480,150:340]=0.
    return rgb,depth,s,ref


class HeldAppearanceTests(unittest.TestCase):
    def setUp(self):
        self.source=template();self.model=build_appearance(*self.source)
        self.rgb,self.depth,self.snapshot,self.reference=current_scene(self.model)
    def assess(self):return assess(self.rgb,self.depth,self.snapshot,self.reference,self.model)
    def test_source_body_and_cap_come_from_real_depth_mask(self):
        self.assertEqual(self.model['source'],'actual_verified_can_rgbd')
        self.assertGreater(self.model['source_body_pixels'],100);self.assertGreater(self.model['source_cap_pixels'],30)
        self.assertFalse(self.model['new_target_identification_supported'])
    def test_projection_matches_independent_recorded_calibration(self):
        p=project_held_can(self.snapshot,self.reference)
        self.assertAlmostEqual(p['cap_camera_depth_prediction_m'],.3780697935083869,places=6)
        np.testing.assert_allclose(p['cap_center_pixel'],[237.60075322798065,337.0895271234626],atol=.001)
    def test_visible_body_and_cap_allow_only_held_rgb_evidence(self):
        result=self.assess();self.assertTrue(result['current_rgb_appearance_verified'])
        self.assertFalse(result['current_depth_surface_verified']);self.assertFalse(result['new_target_identity_created'])
        self.assertFalse(result['measured_3d_object_position_available']);self.assertIsNone(result['measured_center_base_m'])
        self.assertFalse(result['motion_authorized']);self.assertFalse(result['placement_verified'])
    def test_no_prior_verified_grasp_no_rgb_exception(self):
        self.reference['can_grasp_and_lift_verified']=False
        with self.assertRaises(ValueError):self.assess()
    def test_open_or_non_tucked_hand_is_not_held_evidence(self):
        for key,value in (('hand_motor_joint',.65),('arm_flex_joint',-1.7)):
            old=self.snapshot['joints'][key];self.snapshot['joints'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):self.assess()
            self.snapshot['joints'][key]=old
    def test_all_zero_depth_sensor_is_not_claimed_as_near_clipping(self):
        self.depth*=0
        with self.assertRaisesRegex(ValueError,'depth_sensor'):self.assess()
    def test_positive_cap_depth_does_not_use_near_clip_exception(self):
        self.depth[320:360,210:265]=.42
        with self.assertRaisesRegex(ValueError,'actual_rgbd'):self.assess()
    def test_empty_wrist_cannot_pass_even_with_same_grip(self):
        self.rgb[:]=225
        with self.assertRaises(ValueError):self.assess()
    def test_green_body_without_metal_cap_does_not_pass(self):
        color=cv2.cvtColor(np.array([[[self.model['hue'],190,190]]],np.uint8),cv2.COLOR_HSV2RGB)[0,0]
        self.rgb[290:390,170:315]=color
        with self.assertRaisesRegex(ValueError,'cap'):self.assess()
    def test_metal_cap_without_matching_body_does_not_pass(self):
        hsv=cv2.cvtColor(self.rgb,cv2.COLOR_RGB2HSV)
        self.rgb[hsv[:,:,1]>70]=[220,220,220]
        with self.assertRaises(ValueError):self.assess()
    def test_wrong_body_color_is_not_accepted_by_cap_alone(self):
        hsv=cv2.cvtColor(self.rgb,cv2.COLOR_RGB2HSV);mask=hsv[:,:,1]>70
        self.rgb[mask]=[220,30,25]
        with self.assertRaises(ValueError):self.assess()
    def test_matching_object_outside_hand_roi_does_not_pass(self):
        self.rgb=np.roll(self.rgb,240,axis=1)
        with self.assertRaises(ValueError):self.assess()
    def test_valid_far_depth_contradicts_rgb_held_claim(self):
        hsv=cv2.cvtColor(self.rgb,cv2.COLOR_RGB2HSV);self.depth[hsv[:,:,1]>70]=.8
        with self.assertRaisesRegex(ValueError,'contradicts'):self.assess()
    def test_same_color_distractor_without_paired_cap_is_ignored(self):
        color=cv2.cvtColor(np.array([[[self.model['hue'],190,190]]],np.uint8),cv2.COLOR_HSV2RGB)[0,0].tolist()
        cv2.rectangle(self.rgb,(170,450),(305,479),color,-1)
        self.assertTrue(self.assess()['current_rgb_appearance_verified'])
    def test_unrelated_template_class_is_rejected(self):
        rgb,depth,s,m=self.source;m=dict(m,class_name='empty_ketchup')
        with self.assertRaises(ValueError):build_appearance(rgb,depth,s,m)
    def test_template_without_colored_body_is_rejected(self):
        rgb,depth,s,m=self.source
        with self.assertRaises(ValueError):build_appearance(np.full_like(rgb,180),depth,s,m)
    def test_pixels_depth_metadata_are_not_mutated(self):
        rgb=self.rgb.copy();depth=self.depth.copy();snapshot=copy.deepcopy(self.snapshot);ref=copy.deepcopy(self.reference)
        self.assess();np.testing.assert_array_equal(self.rgb,rgb);np.testing.assert_array_equal(self.depth,depth)
        self.assertEqual(self.snapshot,snapshot);self.assertEqual(self.reference,ref)
    def test_default_navigation_entry_never_starts_ros_or_motion(self):
        script=Path(__file__).with_name('run_tucked_can_navigation.py')
        result=subprocess.run([sys.executable,str(script)],capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr);value=json.loads(result.stdout)
        self.assertFalse(value['robot_commands_sent']);self.assertFalse(value['arm_or_gripper_commands_supported'])


if __name__=='__main__':unittest.main()
