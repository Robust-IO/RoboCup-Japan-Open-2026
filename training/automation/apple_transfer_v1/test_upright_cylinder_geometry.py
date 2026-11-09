"""Independent ray/cylinder renders and negative observations; no robot control."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from generic_transfer_template import OfflineBackend,plan,run_offline
from inspect_manipulation_capture import inspect
from manipulation_profiles import object_profile,transfer_config
from object_observation_sequence import ObjectObservationSequence
from object_surface_geometry import estimate
from upright_cylinder_geometry import cylinder_from_points,fit_side_circle,select_depth_surface


def rendered_cylinder(radius=.025,height=.11,color=(30,180,60)):
    width,pixels_high=320,240;f=320.;v,u=np.indices((pixels_high,width))
    theta=.65;s,c=math.sin(theta),math.cos(theta)
    orient=np.array([[0,-s,c],[-1,0,0],[0,-c,-s]])
    rays=np.stack(((u-width/2)/f,(v-pixels_high/2)/f,np.ones_like(u)),axis=-1)@orient.T
    origin=np.array([.1,0.,1.]);center=np.array([.7,0.,.44+height/2])
    offset=origin-center
    a=np.sum(rays[:,:,:2]**2,axis=-1)
    b=2*np.sum(rays[:,:,:2]*offset[:2],axis=-1)
    cc=np.sum(offset[:2]**2)-radius**2
    disc=b*b-4*a*cc
    root=np.sqrt(np.maximum(disc,0))
    candidates=[]
    for t in ((-b-root)/(2*a),(-b+root)/(2*a)):
        z=offset[2]+t*rays[:,:,2]
        candidates.append(np.where((disc>=0)&(t>0)&(np.abs(z)<=height/2),t,np.inf))
    for z in (-height/2,height/2):
        t=(z-offset[2])/rays[:,:,2]
        xy=offset[:2]+t[:,:,None]*rays[:,:,:2]
        candidates.append(np.where((t>0)&(np.sum(xy**2,axis=-1)<=radius**2),t,np.inf))
    surface=np.min(candidates,axis=0)
    table=(.44-origin[2])/rays[:,:,2]
    hit=surface<table;depth=np.where(hit,surface,table)
    rgb=np.full((pixels_high,width,3),[170,130,100],np.uint8);rgb[hit]=color
    yy,xx=np.where(hit);box=[float(xx.min()-2),float(yy.min()-2),float(xx.max()+3),float(yy.max()+3)]
    snapshot=dict(streams=dict(rgb=dict(stamp_ns=1_000_000_000),depth=dict(stamp_ns=1_000_000_000)),
        rgb_k=[f,0,width/2,0,f,pixels_high/2,0,0,1],rgb_d=[0]*5,
        rgb_from_depth=dict(translation=[0,0,0],quaternion=[0,0,0,1]),
        base_from_rgb=dict(translation=origin.tolist(),quaternion=Rotation.from_matrix(orient).as_quat().tolist()))
    detection=dict(name='canned_juice',class_id=3,confidence=.9,box=box)
    return rgb,depth,snapshot,detection,center


class CylinderTests(unittest.TestCase):
    def setUp(self):self.rgb,self.depth,self.snap,self.d,self.center=rendered_cylinder()
    def measure(self,**updates):
        values=dict(rgb=self.rgb,depth=self.depth,snapshot=self.snap,detection=self.d,name='canned_juice')
        values.update(updates);return estimate(**values)
    def test_green_can_has_measured_cylinder_not_sphere_center(self):
        result=self.measure()
        np.testing.assert_allclose(result['center_m'],self.center,atol=.004)
        self.assertAlmostEqual(result['height_m'],.11,delta=.003)
        self.assertAlmostEqual(result['footprint_radius_m'],.025,delta=.002)
        self.assertAlmostEqual(result['estimated_grasp_width_m'],.05,delta=.004)
        self.assertNotAlmostEqual(result['height_m']/2,result['footprint_radius_m'],places=2)
        self.assertIsNone(result['footprint_yaw_rad'])
        for key in ('motion_authorized','identity_confirmed','grasp_center_verified','room_confirmed','ready_for_motion'):
            self.assertFalse(result[key])
    def test_color_does_not_control_can_geometry(self):
        center=self.measure()['center_m']
        for color in ((240,30,20),(0,0,0),(250,250,250)):
            rgb,depth,snapshot,d,_=rendered_cylinder(color=color)
            np.testing.assert_allclose(estimate(rgb,depth,snapshot,d,'canned_juice')['center_m'],center,atol=1e-10)
    def test_cylinder_never_calls_apple_or_cube_fitter(self):
        with patch('apple_rgbd_candidate.estimate',side_effect=AssertionError('apple')),patch('object_surface_geometry.cube_from_points',side_effect=AssertionError('cube')):
            self.assertEqual(self.measure()['shape'],'upright_cylinder')
    def test_depth_zero_and_nan_are_not_object_coordinates(self):
        for depth in (self.depth*0,np.full_like(self.depth,np.nan)):
            with self.assertRaises(ValueError):self.measure(depth=depth)
    def test_wrong_class_is_not_accepted_as_can(self):
        with self.assertRaisesRegex(ValueError,'class_mismatch'):self.measure(detection=dict(self.d,name='apple'))
    def test_expired_sync_and_unregistered_depth(self):
        for kind in ('sync','registration'):
            snapshot=copy.deepcopy(self.snap)
            if kind=='sync':snapshot['streams']['depth']['stamp_ns']+=50_000_000
            else:snapshot['rgb_from_depth']['translation'][0]=.02
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.measure(snapshot=snapshot)
    def test_small_noise_does_not_invent_a_different_center(self):
        rng=np.random.default_rng(41)
        result=self.measure(depth=self.depth+rng.uniform(-.0005,.0005,self.depth.shape))
        np.testing.assert_allclose(result['center_m'],self.center,atol=.004)
    def test_circle_center_from_visible_partial_arc(self):
        angles=np.linspace(-2.8,-.35,300)
        points=np.c_[.7+.025*np.cos(angles),.04+.025*np.sin(angles)]
        center,radius,error,coverage=fit_side_circle(points,[.01,.06])
        np.testing.assert_allclose(center,[.7,.04],atol=1e-8)
        self.assertAlmostEqual(radius,.025);self.assertLess(error,1e-8);self.assertGreater(coverage,2)
    def test_image_touching_background_is_split_by_actual_3d_gap(self):
        v,u=np.indices((30,30));points=np.stack((.8+u*.001,v*.001,np.full_like(u,.55,dtype=float)),axis=-1)
        points[:5,:,0]+=.25
        mask=np.ones((30,30),bool);seeds=np.zeros_like(mask);seeds[10:20,10:20]=True
        selected,detail=select_depth_surface(points,mask,seeds)
        self.assertFalse(selected[:5].any());self.assertTrue(selected[5:].all())
        self.assertEqual(detail['discarded_background_points'],150)
        self.assertEqual(detail['depth_component_count'],2)
    def test_full_rgbd_pipeline_rejects_depth_attached_background_strip(self):
        depth=self.depth.copy()
        x1,y1,x2,y2=self.d['box']
        depth[math.ceil(y1+1):math.ceil(y1+6),math.ceil(x1+1):math.floor(x2-1)]+=.06
        result=self.measure(depth=depth)
        np.testing.assert_allclose(result['center_m'],self.center,atol=.004)
        self.assertGreater(result['surface_segmentation']['discarded_background_points'],0)
    def test_largest_background_is_not_selected_over_center_surface(self):
        v,u=np.indices((30,30));points=np.stack((.8+u*.001,v*.001,np.full_like(u,.55,dtype=float)),axis=-1)
        points[:20,:,0]+=.25
        mask=np.ones((30,30),bool);seeds=np.zeros_like(mask);seeds[22:28,10:20]=True
        selected,detail=select_depth_surface(points,mask,seeds)
        self.assertEqual(int(selected.sum()),300);self.assertFalse(selected[:20].any())
    def test_two_depth_surfaces_crossing_center_are_ambiguous(self):
        v,u=np.indices((30,30));points=np.stack((.8+u*.001,v*.001,np.full_like(u,.55,dtype=float)),axis=-1)
        points[:15,:,0]+=.25
        seeds=np.zeros((30,30),bool);seeds[10:20,10:20]=True
        with self.assertRaisesRegex(ValueError,'ambiguous'):
            select_depth_surface(points,np.ones((30,30),bool),seeds)
    def test_no_seed_surface_cannot_choose_large_background(self):
        points=np.zeros((30,30,3));mask=np.ones((30,30),bool)
        with self.assertRaisesRegex(ValueError,'missing'):
            select_depth_surface(points,mask,np.zeros_like(mask))
    def test_flat_face_and_tiny_arc_cannot_claim_side_curvature(self):
        points=np.c_[np.full(100,.7),np.linspace(-.03,.03,100)]
        with self.assertRaises(ValueError):fit_side_circle(points,[.01,.06])
        angle=np.linspace(0,.05,100);points=np.c_[.7+.025*np.cos(angle),.025*np.sin(angle)]
        with self.assertRaises(ValueError):fit_side_circle(points,[.01,.06])
    def test_low_score_geometry_never_confirms_identity(self):
        self.assertFalse(self.measure(detection=dict(self.d,confidence=.2))['identity_confirmed'])
    def test_observation_keeps_raw_files_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);Image.fromarray(self.rgb).save(root/'rgb.png');np.save(root/'depth_m.npy',self.depth)
            (root/'snapshot.json').write_text(json.dumps(self.snap))
            report=inspect(root,[self.d],'canned_juice')
            self.assertTrue(report['unique_geometry_candidate']);self.assertFalse(report['motion_authorized'])
            np.testing.assert_array_equal(np.load(root/'depth_m.npy'),self.depth)
    def test_profile_and_offline_flow_do_not_enable_live_can_motion(self):
        config=transfer_config('canned_juice')
        self.assertEqual(config['object_profile']['class_id'],3)
        self.assertFalse(config['live_motion_supported'])
        self.assertEqual(plan('canned_juice')['adapter'],'upright_cylinder_rgbd')
        with tempfile.TemporaryDirectory() as folder:
            result=run_offline(config,'can',Path(folder)/'new',OfflineBackend())
            self.assertEqual(len(result['confirmed_stages']),17);self.assertFalse(result['physical_transfer_verified'])
    def test_three_frames_are_map_and_class_bound_not_a_motion_lease(self):
        env=dict(internal_name='LayoutA',rooms=dict(bedroom=dict(region=[[0,-1],[2,-1],[2,1],[0,1]])))
        sequence=ObjectObservationSequence('canned_juice','task','epoch',env,'bedroom')
        for stamp in (1_000_000_000,1_150_000_000,1_300_000_000):
            snapshot=copy.deepcopy(self.snap)
            for key in ('rgb','depth'):snapshot['streams'][key]['stamp_ns']=stamp
            snapshot['tracking_epoch']='epoch'
            snapshot['tracking_transforms']=dict(map_from_base_at_depth=dict(stamp_ns=stamp,translation=[0,0,0],quaternion=[0,0,0,1],source_frame='base_link',target_frame='map'))
            geometry=self.measure(snapshot=snapshot)
            observation=dict(class_name='canned_juice',unique_geometry_candidate=True,geometry_candidates=[dict(detection=self.d,geometry=geometry)])
            result=sequence.observe(observation,snapshot,task_id='task',now_ns=stamp+10_000_000)
        self.assertTrue(result['coarse_identity_consistent']);self.assertFalse(result['motion_authorized'])


if __name__=='__main__':unittest.main()
