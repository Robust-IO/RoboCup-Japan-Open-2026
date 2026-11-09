"""Synthetic template/geometry tests. No ROS connection or real robot backend."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from apple_transfer_automation import STAGES,write_json
from generic_transfer_template import OfflineBackend,plan,run_offline
from manipulation_profiles import BASELINE_PATH,object_profile,transfer_config,validate_transfer_config
from object_surface_geometry import cube_from_points,estimate,support_plane
from inspect_manipulation_capture import inspect
from object_observation_sequence import ObjectObservationSequence


def rendered_cube(yaw=0.,color=(20,130,220),side=.06):
    """Ray/AABB and table-plane intersections, not the fitted algorithm itself."""
    width,height=320,240;f=320.;v,u=np.indices((height,width))
    theta=.65;s,c=math.sin(theta),math.cos(theta)
    orient=np.array([[0,-s,c],[-1,0,0],[0,-c,-s]])
    rays=np.stack(((u-width/2)/f,(v-height/2)/f,np.ones_like(u)),axis=-1)@orient.T
    origin=np.array([.1,0.,1.]);center=np.array([.7,0.,.44+side/2])
    cube_r=Rotation.from_euler('z',yaw).as_matrix()
    local_origin=(origin-center)@cube_r;local_rays=rays@cube_r
    safe=np.where(np.abs(local_rays)<1e-12,1e-12,local_rays)
    a=(-side/2-local_origin)/safe;b=(side/2-local_origin)/safe
    near=np.max(np.minimum(a,b),axis=-1);far=np.min(np.maximum(a,b),axis=-1)
    table=(.44-origin[2])/rays[:,:,2]
    hit=(near>0)&(far>=near)&(near<table)
    depth=np.where(hit,near,table)
    rgb=np.full((height,width,3),[170,130,100],np.uint8);rgb[hit]=color
    yy,xx=np.where(hit)
    box=[float(xx.min()-2),float(yy.min()-2),float(xx.max()+3),float(yy.max()+3)]
    snapshot=dict(streams=dict(rgb=dict(stamp_ns=1_000_000_000),depth=dict(stamp_ns=1_000_000_000)),
                  rgb_k=[f,0,width/2,0,f,height/2,0,0,1],rgb_d=[0]*5,
                  rgb_from_depth=dict(translation=[0,0,0],quaternion=[0,0,0,1]),
                  base_from_rgb=dict(translation=origin.tolist(),quaternion=Rotation.from_matrix(orient).as_quat().tolist()))
    detection=dict(name='rubiks_cube',class_id=16,confidence=.9,box=box)
    return rgb,depth,snapshot,detection,center


class ProfileTests(unittest.TestCase):
    def test_original_apple_config_is_not_changed(self):
        original=json.loads(BASELINE_PATH.read_text());before=copy.deepcopy(original)
        config=transfer_config('rubiks_cube',original)
        self.assertEqual(original,before)
        self.assertEqual(config['baseline'],before)
        self.assertEqual(config['task']['object'],'rubiks_cube')
        self.assertEqual(config['baseline']['task']['object'],'apple')
        self.assertFalse(config['live_motion_supported'])
    def test_unsupported_class_cannot_fall_back_to_apple(self):
        for name in ('salt','filled_ketchup','unknown'):
            with self.subTest(name=name),self.assertRaises(ValueError):object_profile(name)
    def test_profiles_are_independent_copies(self):
        p=object_profile('rubiks_cube');p['side_range_m'][0]=0
        self.assertEqual(object_profile('rubiks_cube')['side_range_m'],[.02,.12])
    def test_altered_route_and_live_flag_are_rejected(self):
        for change in ('route','live','object'):
            config=transfer_config('rubiks_cube')
            if change=='route':config['baseline']['navigation_tolerances']=[.2,.2,.2]
            elif change=='live':config['live_motion_supported']=True
            else:config['task']['object']='apple'
            with self.subTest(change=change),self.assertRaises(ValueError):validate_transfer_config(config)
    def test_plan_is_not_a_live_entry(self):
        result=plan('rubiks_cube')
        self.assertEqual(result['stages'],list(STAGES))
        self.assertFalse(result['robot_commands_sent']);self.assertFalse(result['live_motion_supported'])


class TemplateTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.root=Path(self.directory.name)
        self.addCleanup(self.directory.cleanup)
    def run_template(self,backend,name='rubiks_cube',suffix='run'):
        return run_offline(transfer_config(name),'task',self.root/suffix,backend)
    def test_both_objects_use_same_seventeen_stages_without_physical_success_claim(self):
        for name in ('apple','rubiks_cube','canned_juice'):
            result=self.run_template(OfflineBackend(),name,name)
            self.assertEqual(result['confirmed_stages'],list(STAGES))
            self.assertEqual(result['status'],'offline_flow_verified_not_physical_transfer')
            for key in ('physical_transfer_verified','robot_commands_sent','task_finished_sent','does_not_exist_sent'):
                self.assertFalse(result[key])
    def test_live_backend_is_rejected_before_bind(self):
        class Live(OfflineBackend):
            mode='live'
            def bind(self,*args):raise AssertionError('must not bind')
        with self.assertRaisesRegex(ValueError,'not_enabled'):self.run_template(Live())
        self.assertFalse((self.root/'run').exists())
    def test_each_failure_stops_later_stages_without_retry(self):
        for index,failed in enumerate(STAGES):
            class Broken(OfflineBackend):
                calls=[]
                def perform(self,stage):
                    self.calls.append(stage)
                    if stage==failed:raise RuntimeError('injected')
                    return super().perform(stage)
            backend=Broken();backend.calls=[]
            result=self.run_template(backend,suffix=failed)
            self.assertEqual(backend.calls,list(STAGES[:index+1]))
            self.assertEqual(result['status'],'stopped_offline')
            self.assertFalse(result['automatic_retry'])
    def test_wrong_class_or_old_task_evidence_does_not_advance(self):
        for key,value in [('class_name','apple'),('task_id','old-task'),('synthetic',False),('physical_action_executed',True)]:
            class Wrong(OfflineBackend):
                def perform(self,stage):
                    result=super().perform(stage);path=Path(result['evidence']);data=json.loads(path.read_text())
                    data[key]=value;write_json(path,data);return result
            result=self.run_template(Wrong(),suffix=key)
            self.assertEqual(result['confirmed_stages'],[])
            self.assertEqual(result['status'],'stopped_offline')
    def test_old_output_never_replayed(self):
        self.run_template(OfflineBackend())
        with self.assertRaises(FileExistsError):self.run_template(OfflineBackend())
    def test_evidence_must_belong_to_current_run(self):
        outside=self.root/'outside.json';outside.write_text('{}')
        class Outside(OfflineBackend):
            def perform(self,stage):return dict(verified=True,evidence=str(outside))
        result=self.run_template(Outside());self.assertEqual(result['status'],'stopped_offline')


class CubeGeometryTests(unittest.TestCase):
    def setUp(self):self.rgb,self.depth,self.snap,self.detection,self.center=rendered_cube()
    def measure(self,**changes):
        params=dict(rgb=self.rgb,depth=self.depth,snapshot=self.snap,detection=self.detection,name='rubiks_cube');params.update(changes)
        return estimate(**params)
    def test_real_ray_render_measures_non_red_cube(self):
        result=self.measure()
        np.testing.assert_allclose(result['center_m'],self.center,atol=.006)
        self.assertAlmostEqual(result['height_m'],.06,delta=.004)
        self.assertGreater(result['footprint_radius_m'],result['height_m']/2)
        for key in ('identity_confirmed','grasp_center_verified','motion_authorized','room_confirmed','ready_for_motion'):
            self.assertFalse(result[key])
    def test_colors_do_not_affect_cube_geometry(self):
        baseline=self.measure()
        for color in ((240,20,30),(0,0,0),(250,250,250)):
            rgb,depth,snap,d,_=rendered_cube(color=color)
            np.testing.assert_allclose(estimate(rgb,depth,snap,d,'rubiks_cube')['center_m'],baseline['center_m'],atol=1e-10)
    def test_rotated_cube_is_still_measured(self):
        for yaw in (.25,.7,1.2):
            rgb,depth,snap,d,center=rendered_cube(yaw=yaw)
            with self.subTest(yaw=yaw):np.testing.assert_allclose(estimate(rgb,depth,snap,d,'rubiks_cube')['center_m'],center,atol=.006)
    def test_no_depth_cannot_use_old_center_or_expected_shape(self):
        with self.assertRaises(ValueError):self.measure(depth=self.depth*0)
    def test_nan_depth_does_not_become_valid_geometry(self):
        with self.assertRaises(ValueError):self.measure(depth=np.full_like(self.depth,float('nan')))
    def test_flat_table_is_not_cube(self):
        rgb,depth,snap,d,_=rendered_cube()
        # Remove the cube with ray/plane intersections, not a preselected center.
        from hand_camera_ray import rotation
        v,u=np.indices(depth.shape);k=np.array(snap['rgb_k']).reshape(3,3)
        directions=np.stack(((u-k[0,2])/k[0,0],(v-k[1,2])/k[1,1],np.ones_like(depth)),axis=-1)@rotation(snap['base_from_rgb']['quaternion']).T
        table=(.44-1.)/directions[:,:,2]
        with self.assertRaises(ValueError):self.measure(depth=table)
    def test_synchronization_and_clipped_box(self):
        snap=copy.deepcopy(self.snap);snap['streams']['depth']['stamp_ns']+=40_000_000
        with self.assertRaises(ValueError):self.measure(snapshot=snap)
        with self.assertRaises(ValueError):self.measure(detection=dict(self.detection,box=[0,0,30,30]))
    def test_wrong_class_never_uses_cube_fitter(self):
        with self.assertRaisesRegex(ValueError,'class_mismatch'):self.measure(detection=dict(self.detection,name='apple'))
    def test_bad_intrinsics_quaternion_and_distortion(self):
        for field in ('k','q','distortion'):
            snap=copy.deepcopy(self.snap)
            if field=='k':snap['rgb_k'][0]=0
            elif field=='q':snap['base_from_rgb']['quaternion']=[0,0,0,0]
            else:snap['rgb_d'][0]=.01
            with self.subTest(field=field),self.assertRaises(ValueError):self.measure(snapshot=snap)
    def test_low_score_is_only_a_geometry_proposal(self):
        result=self.measure(detection=dict(self.detection,confidence=.2))
        self.assertFalse(result['identity_confirmed']);self.assertFalse(result['motion_authorized'])
    def test_unregistered_depth_cannot_be_treated_as_rgb_pixels(self):
        for mode in ('missing','translation','intrinsics'):
            snap=copy.deepcopy(self.snap)
            if mode=='missing':del snap['rgb_from_depth']
            elif mode=='translation':snap['rgb_from_depth']['translation'][0]=.02
            else:snap['depth_k']=[100,0,160,0,100,120,0,0,1]
            with self.subTest(mode=mode),self.assertRaises(ValueError):self.measure(snapshot=snap)
    def test_support_fit_is_independent_of_object_center(self):
        x,y=np.meshgrid(np.linspace(.4,.9,40),np.linspace(-.3,.3,40))
        coef,_,_=support_plane(np.c_[x.ravel(),y.ravel(),np.full(x.size,.44)])
        np.testing.assert_allclose(coef,[0,0,.44],atol=1e-8)
    def test_single_flat_vertical_face_cannot_supply_hidden_cube_extent(self):
        y,z=np.meshgrid(np.linspace(-.03,.03,30),np.linspace(.44,.5,30))
        face=np.c_[np.full(y.size,.7),y.ravel(),z.ravel()]
        x,y=np.meshgrid(np.linspace(.4,.9,40),np.linspace(-.3,.3,40))
        plane=np.c_[x.ravel(),y.ravel(),np.full(x.size,.44)]
        with self.assertRaises(ValueError):cube_from_points(face,plane,object_profile('rubiks_cube'))


class AppleCompatibilityTests(unittest.TestCase):
    def test_original_measurement_is_delegated_without_numeric_changes(self):
        from apple_rgbd_candidate import estimate as legacy
        n=120;v,u=np.indices((n,n));x=(u-60)/200;y=(v-60)/200
        a=x*x+y*y+1;disc=1-4*a*(.25-.04**2);hit=disc>0
        depth=np.zeros((n,n));depth[hit]=(1-np.sqrt(disc[hit]))/(2*a[hit])
        rgb=np.zeros((n,n,3),np.uint8);rgb[hit]=[230,30,50]
        snapshot=dict(streams=dict(rgb=dict(stamp_ns=1_000_000_000),depth=dict(stamp_ns=1_000_000_000)),
                      rgb_k=[200,0,60,0,200,60,0,0,1],base_from_rgb=dict(translation=[0,0,0],quaternion=[0,0,0,1]))
        detection=dict(name='apple',confidence=.9,box=[43,43,77,77])
        expected=legacy(rgb,depth,snapshot,detection,geometry_proposal_only=True)
        result=estimate(rgb,depth,snapshot,detection,'apple')
        self.assertEqual(result['legacy_candidate'],expected)
        self.assertEqual(result['adapter'],'legacy_apple_red_sphere')
    def test_cube_cannot_silently_call_apple_geometry(self):
        from unittest.mock import patch
        rgb,depth,snap,d,_=rendered_cube()
        with patch('apple_rgbd_candidate.estimate',side_effect=AssertionError('apple fitter called')):
            self.assertEqual(estimate(rgb,depth,snap,d,'rubiks_cube')['class_name'],'rubiks_cube')


class InspectionTests(unittest.TestCase):
    def test_replay_keeps_pixels_and_depth_unchanged(self):
        rgb,depth,snap,d,_=rendered_cube()
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);Image.fromarray(rgb).save(root/'rgb.png');np.save(root/'depth_m.npy',depth)
            (root/'snapshot.json').write_text(json.dumps(snap))
            result=inspect(root,[d],'rubiks_cube')
            self.assertTrue(result['unique_geometry_candidate']);self.assertFalse(result['motion_authorized'])
            self.assertFalse(result['identity_confirmed']);self.assertFalse(result['does_not_exist_proven'])
            np.testing.assert_array_equal(np.load(root/'depth_m.npy'),depth)
            result=inspect(root,[],'rubiks_cube');self.assertFalse(result['does_not_exist_proven'])
            result=inspect(root,[d,dict(d,box=[x+.01 for x in d['box']])],'rubiks_cube')
            self.assertFalse(result['unique_geometry_candidate'])


class ObservationSequenceTests(unittest.TestCase):
    def setUp(self):
        self.env=dict(internal_name='LayoutA',rooms=dict(bedroom=dict(region=[[0,-1],[2,-1],[2,1],[0,1]])))
        self.track=ObjectObservationSequence('rubiks_cube','task','epoch',self.env,'bedroom')
    def sample(self,stamp=1_000_000_000):
        rgb,depth,snapshot,d,_=rendered_cube()
        snapshot['streams']['rgb']['stamp_ns']=stamp;snapshot['streams']['depth']['stamp_ns']=stamp
        snapshot['tracking_epoch']='epoch'
        snapshot['tracking_transforms']=dict(map_from_base_at_depth=dict(stamp_ns=stamp,translation=[0,0,0],quaternion=[0,0,0,1],source_frame='base_link',target_frame='map'))
        geometry=estimate(rgb,depth,snapshot,d,'rubiks_cube')
        inspection=dict(class_name='rubiks_cube',unique_geometry_candidate=True,geometry_candidates=[dict(detection=d,geometry=geometry)])
        return inspection,snapshot
    def test_three_real_distinct_source_stamps_still_do_not_authorize_motion(self):
        for stamp in (1_000_000_000,1_150_000_000,1_300_000_000):
            inspection,snapshot=self.sample(stamp)
            result=self.track.observe(inspection,snapshot,task_id='task',now_ns=stamp+10_000_000)
        self.assertTrue(result['coarse_identity_consistent']);self.assertFalse(result['motion_authorized'])
    def test_wrong_class_epoch_task_and_expired_snapshot_reset(self):
        for mode in ('class','epoch','task','age','adapter'):
            inspection,snapshot=self.sample();task='task';now=1_010_000_000
            if mode=='class':inspection['class_name']='apple'
            elif mode=='epoch':snapshot['tracking_epoch']='old'
            elif mode=='task':task='old'
            elif mode=='age':now=2_000_000_000
            else:inspection['geometry_candidates'][0]['geometry']['adapter']='legacy_apple_red_sphere'
            result=self.track.observe(inspection,snapshot,task_id=task,now_ns=now)
            self.assertFalse(result['coarse_identity_consistent']);self.assertFalse(result['does_not_exist_proven'])
    def test_duplicate_source_frames_never_count_as_stability(self):
        inspection,snapshot=self.sample()
        for _ in range(4):result=self.track.observe(inspection,snapshot,task_id='task',now_ns=1_010_000_000)
        self.assertFalse(result['coarse_identity_consistent'])
    def test_outside_room_and_low_score_never_confirm_target(self):
        for mode in ('room','score'):
            inspection,snapshot=self.sample()
            if mode=='room':snapshot['tracking_transforms']['map_from_base_at_depth']['translation']=[5,0,0]
            else:inspection['geometry_candidates'][0]['detection']['confidence']=.3
            result=self.track.observe(inspection,snapshot,task_id='task',now_ns=1_010_000_000)
            self.assertFalse(result['coarse_identity_consistent']);self.assertFalse(result['motion_authorized'])


if __name__=='__main__':unittest.main()
