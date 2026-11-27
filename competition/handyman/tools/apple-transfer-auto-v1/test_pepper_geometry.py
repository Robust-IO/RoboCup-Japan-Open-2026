"""Independent ray-rendered shaker, including missing and competing surfaces."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation

from pepper_geometry import estimate_source,measure_near,fit_sphere,lift_follow,surface_extent,near_window
from rgbd_pixel_window import pixel_window
from pepper_hold import palm_points,expected_centers,current_depth,rgb_continuation
from pepper_transfer_automation import confirmed_peppers
from manipulation_profiles import object_profile
from hand_camera_ray import rotation


def scene(*,height_delta=0.,tilt=0.,remove_body=False,remove_cap=False,shift=(0.,0.,0.)):
    width,pixels_high,focal=480,360,420.;v,u=np.indices((pixels_high,width))
    theta=.65;s,c=np.sin(theta),np.cos(theta)
    orient=np.array([[0,-s,c],[-1,0,0],[0,-c,-s]])
    origin=np.array([.1,0.,1.]);rays=np.stack(((u-width/2)/focal,(v-pixels_high/2)/focal,np.ones_like(u)),axis=-1)@orient.T
    body=np.array([.7,0.,.4675+height_delta])+shift
    axis=np.array([np.sin(tilt),0.,np.cos(tilt)]);cap=body+.0325*axis
    candidates=[];colors=[]
    for name,center,radius,color in (('body',body,.0316,[110,95,55]),('cap',cap,.0187,[96,98,96])):
        if name=='body' and remove_body or name=='cap' and remove_cap:continue
        offset=origin-center;aa=np.sum(rays*rays,axis=-1);bb=2*(rays@offset);cc=offset@offset-radius**2
        disc=bb*bb-4*aa*cc
        for t in ((-bb-np.sqrt(np.maximum(0,disc)))/(2*aa),(-bb+np.sqrt(np.maximum(0,disc)))/(2*aa)):
            axial=(origin+t[:,:,None]*rays-center)@axis
            valid=(disc>=0)&(t>0)
            valid&=((axial>=-.0275)&(axial<=.018)) if name=='body' else axial>=0
            candidates.append(np.where(valid,t,np.inf));colors.append(color)
    # Brown shoulder disk, and narrower neck between the wide body and cap.
    if not remove_body:
        t=((body+.018*axis-origin)@axis)/(rays@axis);p=origin+t[:,:,None]*rays
        radial=np.linalg.norm(p-(body+.018*axis),axis=-1)
        candidates.append(np.where((t>0)&(radial<=np.sqrt(.0316**2-.018**2)),t,np.inf));colors.append([110,95,55])
    offset=origin-body;oa=offset@axis;da=rays@axis;op=offset-oa*axis;dp=rays-da[:,:,None]*axis
    aa=np.sum(dp*dp,axis=-1);bb=2*(dp@op);cc=op@op-.013**2;disc=bb*bb-4*aa*cc
    t=(-bb-np.sqrt(np.maximum(0,disc)))/(2*aa);axial=oa+t*da
    candidates.append(np.where((disc>=0)&(t>0)&(axial>.018)&(axial<.0325),t,np.inf));colors.append([110,95,55])
    table=(.44-origin[2])/rays[:,:,2];stack=np.stack(candidates)
    nearest=stack.min(0);part=stack.argmin(0);hit=nearest<table
    depth=np.where(hit,nearest,table);rgb=np.full((*depth.shape,3),[225,220,210],np.uint8)
    for i,color in enumerate(colors):rgb[hit&(part==i)]=color
    yy,xx=np.where(hit);d=dict(name='ground_pepper',confidence=.99,box=[float(xx.min()-2),float(yy.min()-2),float(xx.max()+3),float(yy.max()+3)])
    snapshot=dict(streams={k:dict(stamp_ns=1_000_000_000,width=width,height=pixels_high) for k in ('rgb','depth')},
        rgb_k=[focal,0,width/2,0,focal,pixels_high/2,0,0,1],depth_k=[focal,0,width/2,0,focal,pixels_high/2,0,0,1],rgb_d=[0]*5,
        rgb_from_depth=dict(translation=[0.,0.,0.],quaternion=[0.,0.,0.,1.]),
        base_from_rgb=dict(translation=origin.tolist(),quaternion=Rotation.from_matrix(orient).as_quat().tolist()))
    return rgb,depth,snapshot,d,body,cap


class PepperGeometryTests(unittest.TestCase):
    def setUp(self):self.rgb,self.depth,self.snapshot,self.d,self.body,self.cap=scene()
    def source(self,**updates):
        params=dict(rgb=self.rgb,depth=self.depth,snapshot=self.snapshot,detection=self.d);params.update(updates)
        return estimate_source(**params)
    def test_observed_wide_body_not_midpoint_of_overall_height(self):
        m=self.source();np.testing.assert_allclose(m['center_m'],self.body,atol=.0015)
        self.assertAlmostEqual(m['body_radius_m'],.0316,delta=.001)
        self.assertAlmostEqual(m['cap_radius_m'],.0187,delta=.002)
        self.assertGreater(m['height_m']/2-m['body_above_support_m'],.008)
        self.assertFalse(m['motion_authorized'])
    def test_tracking_uses_new_points_not_prediction(self):
        model=self.source();rgb,dep,s,_,body,cap=scene(height_delta=.1,shift=(.01,-.006,0.))
        result=measure_near(rgb,dep,s,body+[.004,-.003,.002],cap+[.004,-.003,.002],model)
        np.testing.assert_allclose(result['center_m'],body,atol=.002)
        self.assertFalse(result['predicted_center_returned'])
    def test_cropped_geometry_preserves_full_frame_points_and_fits(self):
        model=self.source()
        for tilt,shift in ((0.,(.01,-.006,0.)),(.2,(0.,.02,0.)),(-.15,(.03,-.025,0.))):
            with self.subTest(tilt=tilt):
                rgb,dep,s,_,body,cap=scene(height_delta=.1,tilt=tilt,shift=shift)
                expected_body=body+[.002,0.,.001];expected_cap=cap+[.002,0.,.001]
                _,u,_=near_window(dep.shape,s,[expected_body,expected_cap],
                    [model['body_radius_m']+.012,model['cap_radius_m']+.012])
                self.assertLess(u.size,dep.size/4)
                cropped=measure_near(rgb,dep,s,expected_body,expected_cap,model)
                with patch('pepper_geometry.near_window',side_effect=lambda shape,*args:pixel_window(shape,0,0,shape[1],shape[0])):
                    full=measure_near(rgb,dep,s,expected_body,expected_cap,model)
                for field in ('center_m','cap_center_m','body_radius_m','cap_radius_m','tilt_deg'):
                    np.testing.assert_allclose(cropped[field],full[field],atol=1e-12,rtol=0)
                for field in ('body_fit','cap_fit'):self.assertEqual(cropped[field],full[field])
    def test_outside_image_prediction_has_no_surface(self):
        with self.assertRaisesRegex(ValueError,'surface_not_observed'):
            measure_near(self.rgb,self.depth,self.snapshot,[.7,10,.5],[.7,10,.5325],self.source())
    def test_cap_and_body_measure_a_tilt_instead_of_assuming_vertical(self):
        model=self.source();rgb,d,s,_,b,c=scene(height_delta=.1,tilt=.2)
        m=measure_near(rgb,d,s,b+[.002,0,.001],c+[.002,0,.001],model)
        self.assertAlmostEqual(m['tilt_deg'],np.rad2deg(.2),delta=3.)
    def test_zero_nan_or_missing_body_cannot_be_replaced_by_cap(self):
        model=self.source()
        for invalid in (self.depth*0,np.full_like(self.depth,np.nan)):
            with self.assertRaises(ValueError):measure_near(self.rgb,invalid,self.snapshot,self.body,self.cap,model)
        for kw in ({'remove_body':True},{'remove_cap':True}):
            rgb,d,s,_,b,c=scene(height_delta=.1,**kw)
            with self.subTest(kw=kw),self.assertRaises(ValueError):measure_near(rgb,d,s,b,c,model)
    def test_planar_wrist_cannot_fit_as_small_rounded_surface(self):
        x,y=np.meshgrid(np.linspace(-.02,.02,20),np.linspace(-.015,.015,20))
        p=np.c_[x.ravel(),y.ravel(),np.full(x.size,.5)]
        with self.assertRaisesRegex(ValueError,'curvature'):fit_sphere(p,[.008,.03])
    def test_no_depth_wrong_class_and_bad_registration_source_fail(self):
        with self.assertRaises(ValueError):self.source(depth=self.depth*0)
        with self.assertRaises(ValueError):self.source(detection=dict(self.d,name='ground_salt'))
        changed=copy.deepcopy(self.snapshot);changed['rgb_from_depth']['translation']=[.01,0,0]
        with self.assertRaises(ValueError):self.source(snapshot=changed)
    def test_source_inputs_are_not_changed(self):
        rgb=self.rgb.copy();depth=self.depth.copy();snapshot=copy.deepcopy(self.snapshot)
        self.source();np.testing.assert_array_equal(rgb,self.rgb);np.testing.assert_array_equal(depth,self.depth)
        self.assertEqual(snapshot,self.snapshot)
    def test_bottom_comes_from_observed_base_not_full_sphere(self):
        result=surface_extent(np.array([.7,0,.5275]),np.array([.7,0,.56]),.0316,.0187,.0275)
        self.assertAlmostEqual(result['estimated_bottom_z_m'],.5)
        self.assertNotAlmostEqual(result['bottom_vertical_extent_m'],.0316)
    def test_actual_palm_rotation_transports_both_parts_during_retraction(self):
        palm=dict(translation=[.6,.078,.55],quaternion=[0,0,0,1])
        body=np.array([.68,.078,.55]);cap=body+[0,0,.0325]
        bp,cp=palm_points(body,cap,palm)
        ref=dict(class_name='ground_pepper',grasp_lift_verified=True,body_palm_m=bp,cap_palm_m=cp,model=self.source())
        rotated=dict(translation=[.3,.078,.7],quaternion=Rotation.from_euler('y',.7).as_quat().tolist())
        b,c=expected_centers(rotated,ref)
        np.testing.assert_allclose(c-b,rotation(rotated['quaternion'])@np.array([0,0,.0325]),atol=1e-12)
    def test_lift_requires_both_actual_surfaces_to_follow_palm(self):
        before=dict(center_m=[.7,0,.5],cap_center_m=[.7,0,.5325],stamp_ns=1_000_000_000)
        def snap(stamp,lift,palm_z):return dict(tracking_epoch='task',streams=dict(depth=dict(stamp_ns=stamp)),
            palm_base_at_depth=[.62,0,palm_z],joints=dict(arm_lift_joint=lift,hand_motor_joint=.4),
            tracking_transforms=dict(odom_from_base_at_depth=dict(stamp_ns=stamp,source_frame='base_link',target_frame='odom',
                translation=[0,0,0],quaternion=[0,0,0,1])))
        bs=snap(1_000_000_000,.22,.5);ass=snap(2_000_000_000,.32,.6)
        after=dict(before,center_m=[.7,0,.6],cap_center_m=[.7,0,.6325],stamp_ns=2_000_000_000)
        self.assertTrue(lift_follow(before,bs,after,ass,.4)['grasp_lift_verified'])
        after['cap_center_m']=before['cap_center_m']
        with self.assertRaisesRegex(ValueError,'follow'):lift_follow(before,bs,after,ass,.4)
    def test_low_score_geometry_is_not_high_confidence_identity(self):
        m=self.source(detection=dict(self.d,confidence=.2));self.assertFalse(m['motion_authorized'])
        self.assertEqual(confirmed_peppers([dict(self.d,confidence=.79)]),[])
    def test_duplicate_boxes_do_not_boost_confidence_or_merge_two_objects(self):
        self.assertEqual(len(confirmed_peppers([self.d,dict(self.d,confidence=.9)])),1)
        other=dict(self.d,box=[x+60 if i%2==0 else x for i,x in enumerate(self.d['box'])])
        self.assertEqual(len(confirmed_peppers([self.d,other])),2)
    def test_profile_keeps_the_canonical_class_id(self):
        self.assertEqual(object_profile('ground_pepper')['class_id'],11)
    def clipped_held(self,**scene_changes):
        orient=rotation(self.snapshot['base_from_rgb']['quaternion']);origin=np.asarray(self.snapshot['base_from_rgb']['translation'])
        center=origin+orient[:,2]*.32
        rgb,depth,s,_,body,cap=scene(shift=center-self.body,**scene_changes)
        depth[depth<.4]=0.
        s['joints']=dict(arm_lift_joint=0.,arm_flex_joint=0.,arm_roll_joint=0.,wrist_flex_joint=-np.pi/2,
            wrist_roll_joint=0.,hand_motor_joint=.4)
        palm=dict(translation=(body-[.08,0,0]).tolist(),quaternion=[0,0,0,1])
        bp,cp=palm_points(body,cap,palm)
        ref=dict(class_name='ground_pepper',grasp_lift_verified=True,body_palm_m=bp,cap_palm_m=cp,
            model=self.source(),holding_opening_rad=.4)
        return rgb,depth,s,palm,ref
    def test_already_held_near_clip_rgb_has_no_fabricated_xyz(self):
        rgb,d,s,palm,ref=self.clipped_held()
        proof=rgb_continuation(rgb,d,s,palm,ref)
        self.assertTrue(proof['current_rgb_appearance_verified']);self.assertFalse(proof['current_depth_surface_verified'])
        self.assertFalse(proof['measured_3d_object_position_available']);self.assertIsNone(proof['measurement'])
    def test_current_background_or_missing_body_cannot_prove_a_clipped_hold(self):
        rgb,d,s,palm,ref=self.clipped_held()
        with self.assertRaises(ValueError):rgb_continuation(np.full_like(rgb,220),d,s,palm,ref)
        rgb,d,s,palm,ref=self.clipped_held(remove_body=True)
        with self.assertRaises(ValueError):rgb_continuation(rgb,d,s,palm,ref)
    def test_rgb_hold_requires_actual_verified_grasp_and_current_grip(self):
        rgb,d,s,palm,ref=self.clipped_held();ref['grasp_lift_verified']=False
        with self.assertRaises(ValueError):rgb_continuation(rgb,d,s,palm,ref)
    def test_default_cli_is_no_motion_plan(self):
        p=Path(__file__).with_name('run_object_transfer.py')
        result=subprocess.run([sys.executable,str(p),'--object','ground_pepper'],capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr)
        value=json.loads(result.stdout);self.assertEqual(value['class_name'],'ground_pepper');self.assertFalse(value['robot_commands_sent'])

if __name__=='__main__':unittest.main()
