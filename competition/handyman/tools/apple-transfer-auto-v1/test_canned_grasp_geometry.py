import copy
import json
from pathlib import Path
import subprocess
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch

import numpy as np

from canned_grasp_geometry import (aligned,anchor,closure_error,contact_offset,contact_point,fresh,
    grasp_plan,lift_follow,measure_near,predicted_before_contact,transform_point,upright_trial_evidence,
    CONTACT_OFFSET_BASE,CLOSE_TOLERANCE)
from test_upright_cylinder_geometry import rendered_cylinder
from canned_grasp_runtime import CanRuntime


def shape(center=(.6892952185,.078,.558),stamp=1_000_000_000):
    return dict(class_name='canned_juice',shape='upright_cylinder',adapter='upright_cylinder_rgbd',
        center_m=list(center),height_m=.125,footprint_radius_m=.0375,stamp_ns=stamp)


def snapshot(stamp=1_000_000_000,lift=.2808021848314039,palm_height=.558,opening=.65):
    return dict(tracking_epoch='can-test',streams={k:dict(stamp_ns=stamp) for k in ('rgb','depth')},
        joint_stamp_ns=stamp,palm_base_at_depth=[.6271693484917622,.078,palm_height],
        joints=dict(arm_lift_joint=lift,hand_motor_joint=opening),
        tracking_transforms=dict(odom_from_base_at_depth=dict(translation=[0,0,0],
            quaternion=[0,0,0,1],stamp_ns=stamp,source_frame='base_link',target_frame='odom')))


class CanGraspTests(unittest.TestCase):
    def test_plan_keeps_half_height_separate_from_footprint_radius(self):
        value=grasp_plan(shape())
        self.assertEqual(value['half_height_m'],.0625)
        self.assertEqual(value['radius_m'],.0375)
        self.assertAlmostEqual(value['bottom_height_m'],.4955)
        self.assertAlmostEqual(value['contact_start_lift_m'],.2808021848314039)
        self.assertFalse(value['motion_authorized'])
    def test_wrong_shape_or_unreachable_old_preparation_is_not_renamed_apple(self):
        for value in (dict(shape(),class_name='apple'),dict(shape(),shape='sphere'),shape(center=(.7,0,.8))):
            with self.assertRaises(ValueError):grasp_plan(value)
    def test_anchor_is_an_odom_roi_not_a_replacement_depth_measurement(self):
        first=snapshot();ref=anchor(shape(),first,task_id='new-task')
        second=snapshot();second['tracking_transforms']['odom_from_base_at_depth']['translation']=[.13,.03,0]
        np.testing.assert_allclose(predicted_before_contact(ref,second,'new-task'),[.5592952185,.048,.558])
        with self.assertRaises(ValueError):predicted_before_contact(ref,second,'other-task')
        second['tracking_epoch']='reset'
        with self.assertRaises(ValueError):predicted_before_contact(ref,second,'new-task')
    def test_image_time_transform_round_trip_with_base_rotation(self):
        s=snapshot();s['tracking_transforms']['odom_from_base_at_depth'].update(translation=[2,-3,.1],quaternion=[0,0,np.sin(.3),np.cos(.3)])
        point=[.7,.08,.55]
        np.testing.assert_allclose(transform_point(s,transform_point(s,point),inverse=True),point,atol=1e-10)
    def test_transform_at_different_image_time_is_not_accepted(self):
        s=snapshot();s['tracking_transforms']['odom_from_base_at_depth']['stamp_ns']+=1
        with self.assertRaises(ValueError):transform_point(s,[.7,0,.5])
    def test_near_fit_uses_actual_cap_and_side_not_rgb_color(self):
        rgb,depth,s,d,center=rendered_cylinder()
        value=measure_near(depth,s,center,.025,.11)
        np.testing.assert_allclose(value['center_m'],center,atol=.003)
        self.assertAlmostEqual(value['footprint_radius_m'],.025,delta=.002)
        self.assertFalse(value['detector_reran']);self.assertFalse(value['hidden_bottom_measured'])
    def test_near_fit_does_not_return_predicted_center(self):
        rgb,depth,s,d,center=rendered_cylinder()
        value=measure_near(depth,s,center+[.006,-.004,.006],.025,.11)
        np.testing.assert_allclose(value['center_m'],center,atol=.003)
        self.assertGreater(np.linalg.norm(np.array(value['center_m'])-(center+[.006,-.004,.006])),.004)
    def test_lower_body_occlusion_still_uses_observed_upper_curve(self):
        from hand_camera_ray import rotation
        rgb,depth,s,d,center=rendered_cylinder()
        v,u=np.indices(depth.shape);k=np.asarray(s['rgb_k']).reshape(3,3)
        points=np.stack(((u-k[0,2])*depth/k[0,0],(v-k[1,2])*depth/k[1,1],depth),axis=-1)
        points=points@rotation(s['base_from_rgb']['quaternion']).T+s['base_from_rgb']['translation']
        depth[(rgb[:,:,1]==180)&(points[:,:,2]<.44+.65*.11)]=0
        value=measure_near(depth,s,center,.025,.11)
        np.testing.assert_allclose(value['center_m'],center,atol=.003)
        self.assertEqual(value['observed_side_height_fraction'],[.7,.95])
    def test_zero_depth_and_missing_can_cannot_be_verified(self):
        rgb,depth,s,d,center=rendered_cylinder()
        for actual in (np.zeros_like(depth),np.full_like(depth,np.nan)):
            with self.assertRaises(ValueError):measure_near(actual,s,center,.025,.11)
        with self.assertRaises(ValueError):measure_near(depth,s,center+[.15,0,0],.025,.11)
    def test_object_left_on_table_is_not_fabricated_at_lift_prediction(self):
        rgb,depth,s,d,center=rendered_cylinder()
        with self.assertRaises(ValueError):measure_near(depth,s,center+[0,0,.1],.025,.11)
    def test_freshness_rejects_stale_future_unsynced_or_reset(self):
        fresh(snapshot(),1_100_000_000,'can-test')
        for kind in ('stale','future','sync','joint','epoch'):
            s=snapshot();now=1_100_000_000;epoch='can-test'
            if kind=='stale':now+=500_000_000
            if kind=='future':now=900_000_000
            if kind=='sync':s['streams']['rgb']['stamp_ns']+=40_000_000
            if kind=='joint':s['joint_stamp_ns']-=500_000_000
            if kind=='epoch':epoch='other'
            with self.subTest(kind=kind),self.assertRaises(ValueError):fresh(s,now,epoch)
    def test_contact_point_uses_actual_palm_not_nominal_apple_center(self):
        s=snapshot();target=contact_point(s).copy();self.assertTrue(aligned(target,s))
        s['palm_base_at_depth'][1]+=.04
        self.assertFalse(aligned(target,s))
        np.testing.assert_allclose(closure_error(target,s),[0,-.04,0],atol=1e-9)
    def test_old_success_offset_does_not_define_the_finger_midline(self):
        s=snapshot();old_offcenter=[.7066554264393505,.09014450403069171,.5551526564303343]
        self.assertFalse(aligned(old_offcenter,s))
        centered=[.7066554264393505,.07795,.5551526564303343]
        self.assertTrue(aligned(centered,s))
        s['palm_base_at_depth'][0]+=.15
        self.assertFalse(aligned(centered,s))
    def test_centering_tolerance_leaves_clearance_inside_observed_open_fingers(self):
        # Independent opening-.65 trigger inner faces, transformed by the
        # observed horizontal palm quaternion. Can radius is measured ~37.5mm.
        inner_low=-.04656304840152277;inner_high=.04646304840152277;radius=.0375
        for error in (-CLOSE_TOLERANCE[1],CLOSE_TOLERANCE[1]):
            center=CONTACT_OFFSET_BASE[1]+error
            self.assertGreater(center-radius,inner_low)
            self.assertLess(center+radius,inner_high)
        self.assertAlmostEqual(CONTACT_OFFSET_BASE[1],(inner_low+inner_high)/2)
    def test_deeper_seating_advances_base_without_changing_side_or_height(self):
        s=snapshot();target=contact_point(s)
        np.testing.assert_allclose(closure_error(target,s,extra_seat_m=.035),[.035,0,0],atol=1e-12)
        self.assertFalse(aligned(target,s,extra_seat_m=.035))
        # Moving the base forward by 35mm makes this stationary can 35mm nearer.
        self.assertTrue(aligned(target-[.035,0,0],s,extra_seat_m=.035))
    def test_deeper_seating_is_opt_in_and_does_not_mutate_calibration(self):
        original=contact_offset();changed=contact_offset(.035)
        np.testing.assert_allclose(changed,original-[.035,0,0],atol=1e-12)
        np.testing.assert_array_equal(contact_offset(),original)
        for invalid in (-.01,.051,float('inf'),float('nan')):
            with self.subTest(invalid=invalid),self.assertRaises(ValueError):contact_offset(invalid)
    def test_upright_experiment_rejects_actual_tipping_before_closure(self):
        def measured(degrees):return dict(class_name='canned_juice',
            position_source='current_cap_plane_and_source_colored_side_depth',
            axis_base_m=[0.,np.sin(np.deg2rad(degrees)),np.cos(np.deg2rad(degrees))])
        self.assertTrue(upright_trial_evidence(measured(.46))['upright_trial_verified'])
        with self.assertRaisesRegex(ValueError,'tipped_stop_before_closure'):
            upright_trial_evidence(measured(24.33))
    def test_upright_experiment_requires_measured_axis_not_an_assumed_label(self):
        with self.assertRaisesRegex(ValueError,'measured_current_can_axis'):
            upright_trial_evidence(shape())
    def test_lift_follows_actual_palm_without_exact_ten_cm_requirement(self):
        before=shape();bs=snapshot()
        after=shape(center=(.6892952185,.078,.638),stamp=2_000_000_000)
        ass=snapshot(2_000_000_000,lift=.3608021848314039,palm_height=.638,opening=.52)
        proof=lift_follow(before,bs,after,ass,.52)
        self.assertAlmostEqual(proof['object_delta_odom_m'][2],.08)
        self.assertTrue(proof['functional_lift_follow_verified']);self.assertFalse(proof['placement_verified'])
    def test_empty_hand_only_rises_cannot_prove_grasp(self):
        bs=snapshot();ass=snapshot(2_000_000_000,lift=.3808021848,palm_height=.658,opening=.52)
        with self.assertRaisesRegex(ValueError,'leave_table'):
            lift_follow(shape(),bs,shape(stamp=2_000_000_000),ass,.52)
    def test_slip_and_open_hand_are_rejected(self):
        bs=snapshot();ass=snapshot(2_000_000_000,lift=.3808021848,palm_height=.658,opening=.52)
        for kind in ('slip','open','radius','time','epoch'):
            changed=copy.deepcopy(ass);after=shape(center=(.6892952185,.078,.658),stamp=2_000_000_000)
            if kind=='slip':after['center_m'][0]+=.04
            if kind=='open':changed['joints']['hand_motor_joint']=.65
            if kind=='radius':after['footprint_radius_m']=.05
            if kind=='time':after['stamp_ns']=1_000_000_000
            if kind=='epoch':changed['tracking_epoch']='reset'
            with self.subTest(kind=kind),self.assertRaises(ValueError):lift_follow(shape(),bs,after,changed,.52)
    def test_default_cli_never_starts_ros_or_motion(self):
        source=Path(__file__).with_name('run_canned_grasp_trial.py')
        result=subprocess.run([sys.executable,str(source)],capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr)
        report=json.loads(result.stdout)
        self.assertEqual(report['status'],'dry_plan');self.assertFalse(report['robot_commands_sent'])
        self.assertEqual(report['maximum_new_measured_corrections'],5)
        self.assertEqual(report['close_commands_maximum'],1)
    def fake_runtime(self):
        runtime=CanRuntime.__new__(CanRuntime);runtime.epoch='can-test';runtime.rejected_serial=0
        runtime.sample=Mock(return_value=dict(snapshot=snapshot(),depth=np.zeros((3,3))))
        runtime.save=Mock(return_value=Path('/tmp/unused-new-depth-rejection'))
        runtime.record=Mock()
        runtime.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_100_000_000)))
        return runtime,anchor(shape(),snapshot(),task_id='new-task')
    def test_one_bad_frame_reacquires_without_motion_or_threshold_change(self):
        runtime,reference=self.fake_runtime()
        with patch('canned_grasp_runtime.measure_near',side_effect=[ValueError('cylinder_side_fit_residual'),shape()]):
            sample,value=runtime.observe_near(reference,'new-task')
        self.assertEqual(runtime.sample.call_count,2);self.assertEqual(runtime.save.call_count,1)
        self.assertEqual(value['class_name'],'canned_juice')
        self.assertEqual(runtime.record.call_args.args[0]['event'],'discard_depth_frame_no_action_replayed')
    def test_persistent_bad_depth_stops_after_five_new_frames(self):
        runtime,reference=self.fake_runtime()
        with patch('canned_grasp_runtime.measure_near',side_effect=ValueError('no_surface')):
            with self.assertRaisesRegex(ValueError,'not_resolved'):
                runtime.observe_near(reference,'new-task')
        self.assertEqual(runtime.sample.call_count,5);self.assertEqual(runtime.save.call_count,5)
    def test_postclose_pose_records_tilt_without_a_new_pose_acceptance_gate(self):
        runtime,reference=self.fake_runtime();reference['source_appearance']={'source':'actual_verified_can_rgbd'}
        runtime.sample.return_value['rgb']=np.zeros((3,3,3),np.uint8)
        runtime.publisher=Mock(side_effect=AssertionError('diagnostic must not publish'))
        with patch('canned_grasp_runtime.measure_near',return_value=dict(shape(),tilt_deg=12.)):
            rows=runtime.record_closed_pose(reference,'new-task')
        self.assertEqual(len(rows),3)
        self.assertTrue(all(row['pose_measured'] and row['tilt_deg']==12. for row in rows))
        runtime.publisher.assert_not_called()
    def test_postclose_fit_failure_keeps_raw_evidence_without_blocking_lift(self):
        runtime,reference=self.fake_runtime();reference['source_appearance']={'source':'actual_verified_can_rgbd'}
        runtime.sample.return_value['rgb']=np.zeros((3,3,3),np.uint8)
        with patch('canned_grasp_runtime.measure_near',side_effect=ValueError('cap_occluded')):
            rows=runtime.record_closed_pose(reference,'new-task')
        self.assertEqual(runtime.save.call_count,3)
        self.assertTrue(all(not row['pose_measured'] and row['measurement_error']=='cap_occluded' for row in rows))
    def test_task_change_is_not_hidden_as_transient_geometry_failure(self):
        runtime,reference=self.fake_runtime()
        with self.assertRaisesRegex(ValueError,'context_mismatch'):
            runtime.observe_near(reference,'different-task')
        self.assertEqual(runtime.sample.call_count,1);runtime.save.assert_not_called()
    def test_radius_failure_uses_current_rgbd_only_with_verified_source_appearance(self):
        runtime,reference=self.fake_runtime();reference['source_appearance']={'source':'actual_verified_can_rgbd'}
        sample=runtime.sample.return_value;sample['rgb']=np.zeros((3,3,3),np.uint8)
        with patch('canned_grasp_runtime.measure_near',side_effect=[
                ValueError('cylinder_radius_outside_observation_envelope'),shape()]) as measure:
            _,value=runtime.observe_near(reference,'new-task')
        self.assertEqual(runtime.sample.call_count,1);runtime.save.assert_not_called()
        self.assertIs(measure.call_args.kwargs['rgb'],sample['rgb'])
        self.assertIs(measure.call_args.kwargs['appearance'],reference['source_appearance'])
        self.assertEqual(value['precontact_upright_fit_rejection'],'cylinder_radius_outside_observation_envelope')
    def test_no_fabricated_surface_when_both_near_fits_fail(self):
        runtime,reference=self.fake_runtime();reference['source_appearance']={'source':'actual_verified_can_rgbd'}
        runtime.sample.return_value['rgb']=np.zeros((3,3,3),np.uint8)
        with patch('canned_grasp_runtime.measure_near',side_effect=[
                ValueError('cylinder_radius_outside_observation_envelope'),
                ValueError('current_metal_cap_depth_not_observed')]*5):
            with self.assertRaisesRegex(ValueError,'current_metal_cap_depth_not_observed'):
                runtime.observe_near(reference,'new-task')
        self.assertEqual(runtime.sample.call_count,5);self.assertEqual(runtime.save.call_count,5)


if __name__=='__main__':unittest.main()
