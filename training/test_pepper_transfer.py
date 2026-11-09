import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock,patch
import numpy as np

from pepper_place import region,expected_placed,verify_placed
from pepper_runtime import PepperRuntime
from pepper_geometry import surface_extent
from test_canned_place_geometry import fixture


class PepperPlacementTests(unittest.TestCase):
    def setUp(self):
        _,_,self.s,_,self.plan,self.plane=fixture()
        self.plan.update(class_name='ground_pepper',body_above_support_m=.0275,height_m=.0787,
            cap_minus_body_horizontal_m=[0.,0.,.0325],actual_release_xy_map_m=[.7,0.])
        self.m=dict(class_name='ground_pepper',center_m=[.7,0.,.4875],cap_center_m=[.7,0.,.52],
            stamp_ns=self.s['streams']['depth']['stamp_ns'],horizontal_support_radius_m=.0316,
            estimated_lowest_point_base_m=[.7,0.,.46])
    def test_release_gap_uses_wide_body_base_offset(self):
        check=region(self.m,self.s,self.plan,self.plane)
        self.assertAlmostEqual(check['estimated_bottom_gap_m'],.02)
        self.assertTrue(check['ready'])
        self.assertFalse(check['planned_xy_is_success_gate'])
    def test_unknown_table_support_and_edge_are_not_release_regions(self):
        self.plan['observed_table_support']['mask_rows']=['0'*201]*201
        self.assertFalse(region(self.m,self.s,self.plan,self.plane)['ready'])
    def test_other_class_or_old_surface_stamp_cannot_be_reused(self):
        for change in ('class_name','stamp_ns'):
            m=copy.deepcopy(self.m);m[change]='canned_juice' if change=='class_name' else m[change]+1
            with self.subTest(change=change),self.assertRaises(ValueError):region(m,self.s,self.plan,self.plane)
    def test_postrelease_roi_uses_actual_release_and_observed_body_base(self):
        body,cap=expected_placed(self.s,self.plan)
        np.testing.assert_allclose(body,[.7,0.,.4675])
        np.testing.assert_allclose(cap,[.7,0.,.5])
    def test_three_stable_supported_frames_verify_the_actual_object(self):
        rows=[]
        for index in range(3):
            s=copy.deepcopy(self.s);m=copy.deepcopy(self.m);plane=copy.deepcopy(self.plane)
            stamp=m['stamp_ns']+index*200_000_000;s['streams']['depth']['stamp_ns']=stamp
            s['streams']['rgb']['stamp_ns']=stamp;s['tracking_transforms']['map_from_base_at_depth']['stamp_ns']=stamp
            m.update(stamp_ns=stamp,center_m=[.7,0.,.4675],estimated_lowest_point_base_m=[.7,0.,.44])
            plane['source_depth_stamp_ns']=stamp
            rows.append(dict(measurement=m,snapshot=s,plane=plane))
        proof=verify_placed(rows,self.plan,[0.,0.,0.,-math.pi/2,0.,.65],cleanup_completed=True)
        self.assertTrue(proof['placement_success_verified'])
        self.assertFalse(proof['task_finished_sent'])


class PepperArmTests(unittest.TestCase):
    def test_visual_direction_compensation_preserves_one_full_motion(self):
        r=PepperRuntime.__new__(PepperRuntime);angle=.14
        s=dict(tracking_epoch='task',streams=dict(depth=dict(stamp_ns=2)))
        e=dict(controller_to_visual_angle_rad=angle,reference_epoch='task',current_depth_stamp_ns=2)
        r.accept_visual_motion(e,s)
        for desired in ([.30,0.,0.],[0.,-.04,0.]):
            command=r.controller_delta(desired);c,t=math.cos(angle),math.sin(angle)
            physical=[c*command[0]-t*command[1],t*command[0]+c*command[1]]
            np.testing.assert_allclose(physical,desired[:2],atol=1e-12)
            self.assertAlmostEqual(math.hypot(*command[:2]),math.hypot(*desired[:2]))
    def test_motion_compensation_is_not_created_from_an_old_or_unrelated_proof(self):
        s=dict(tracking_epoch='task',streams=dict(depth=dict(stamp_ns=2)))
        for values in (dict(reference_epoch='other',current_depth_stamp_ns=2,controller_to_visual_angle_rad=.1),
                       dict(reference_epoch='task',current_depth_stamp_ns=1,controller_to_visual_angle_rad=.1),
                       dict(reference_epoch='task',current_depth_stamp_ns=2,controller_to_visual_angle_rad=float('nan'))):
            r=PepperRuntime.__new__(PepperRuntime);r.accept_visual_motion(values,s)
            self.assertEqual(r.controller_delta([.3,0.,0.]),[.3,0.,0.])
    def test_readonly_placement_verification_requires_current_empty_tucked_pose(self):
        from pepper_transfer_automation import validate_released_placement
        old_state=dict(joints=[.214,-1.7,0.,.1292,0.,.65])
        previous=dict(status='stopped_requires_actual_state_review',class_name='ground_pepper',
            stage='verify_actual_placement',release_commands_sent=1,hand_released=True,
            grasp_success_verified=True,last_fresh_robot_state=old_state)
        meta=dict(boot_id='same');held=dict(boot_id='same',class_name='ground_pepper')
        plan=dict(class_name='ground_pepper',actual_release_xy_map_m=[1.,3.])
        ready=dict(joints=[0.,0.,0.,-math.pi/2,0.,.65])
        validate_released_placement(previous,meta,held,plan,ready,'same',verification_only=True)
        with self.assertRaises(ValueError):validate_released_placement(previous,meta,held,plan,old_state,'same',verification_only=True)

    def test_open_hand_motion_does_not_require_exact_same_settling_grip_value(self):
        from pepper_runtime import arm_goal_with_current_hand
        goal=[.294,-1.7,0.,.1292,0.,.633]
        actual=[.214,-1.7,0.,.1292,0.,.65]
        adjusted=arm_goal_with_current_hand(goal,actual)
        self.assertEqual(adjusted[:5],goal[:5]);self.assertEqual(adjusted[5],.65)
        self.assertEqual(goal[5],.633)
        with self.assertRaisesRegex(ValueError,'preserve_actual_grip'):
            arm_goal_with_current_hand(goal[:5]+[.4],actual[:5]+[.46])
    def test_released_continuation_cannot_reuse_a_held_or_unreleased_state(self):
        from pepper_transfer_automation import validate_released_placement
        state=dict(joints=[.214,-1.7,0.,.1292,0.,.65])
        previous=dict(status='stopped_requires_actual_state_review',class_name='ground_pepper',
            stage='verify_actual_placement',release_commands_sent=1,hand_released=True,
            grasp_success_verified=True,last_fresh_robot_state=state)
        meta=dict(boot_id='same');held=dict(boot_id='same',class_name='ground_pepper')
        plan=dict(class_name='ground_pepper',actual_release_xy_map_m=[1.,3.])
        validate_released_placement(previous,meta,held,plan,state,'same')
        for old in (dict(previous,release_commands_sent=0),dict(previous,hand_released=False)):
            with self.assertRaises(ValueError):validate_released_placement(old,meta,held,plan,state,'same')
        with self.assertRaises(ValueError):validate_released_placement(previous,meta,held,plan,dict(joints=state['joints'][:5]+[.4]),'same')

    def test_measured_bridge_joint_delay_has_a_bounded_class_specific_allowance(self):
        from canned_carry_geometry import current_images_and_feedback
        from canned_grasp_geometry import fresh
        from synced_capture_selection import select_stamps
        from inspect_transfer_ready import validate_sample_times
        now=10_000_000_000;stamps=(now-210_000_000,now-200_000_000,now-120_000_000)
        history={key:{stamp:None} for key,stamp in zip(('rgb','depth','hand'),stamps)}
        kwargs=dict(ready=lambda _:True,joint_stamp_ns=now-474_000_000,
            processing_reserve_ns=100_000_000,joint_processing_reserve_ns=0)
        self.assertIsNone(select_stamps(history,now,**kwargs))
        self.assertEqual(select_stamps(history,now,joint_max_age_ns=600_000_000,**kwargs),stamps)
        s=dict(tracking_epoch='pepper',joint_stamp_ns=now-474_000_000,
            streams=dict(rgb=dict(stamp_ns=stamps[0]),depth=dict(stamp_ns=stamps[1])),joints={})
        fresh(s,now,'pepper',joint_max_age_ns=600_000_000)
        with self.assertRaises(ValueError):fresh(s,now,'pepper')
        for age in (601_000_000,824_000_000):
            old=dict(s,joint_stamp_ns=now-age)
            with self.assertRaises(ValueError):fresh(old,now,'pepper',joint_max_age_ns=600_000_000)
        old=copy.deepcopy(s);old['streams']['rgb']['stamp_ns']=now-401_000_000
        with self.assertRaises(ValueError):fresh(old,now,'pepper',joint_max_age_ns=600_000_000)
        self.assertAlmostEqual(validate_sample_times(now-474_000_000,now-50_000_000,now,joint_max_s=.6)['joints'],.474)
        with self.assertRaises(RuntimeError):validate_sample_times(now-474_000_000,now-50_000_000,now)

    def test_destination_continuation_requires_actual_unreleased_verified_hold(self):
        from pepper_transfer_automation import validate_destination_hold
        previous=dict(status='stopped_requires_actual_state_review',class_name='ground_pepper',
            stage='measure_destination_and_plan',grasp_success_verified=True,release_commands_sent=0,
            confirmed_stages=['grasp_lift','carry_destination'])
        meta=dict(boot_id='same',run_id='source',run_root='/mnt/d/handyman-training-runs/source')
        held=dict(meta,class_name='ground_pepper',grasp_lift_verified=True,holding_opening_rad=.4)
        state=dict(joints=[0.,0.,0.,-math.pi/2,0.,.4])
        validate_destination_hold(previous,meta,held,state,'same')
        for old in (dict(previous,release_commands_sent=1),dict(previous,grasp_success_verified=False)):
            with self.assertRaises(ValueError):validate_destination_hold(old,meta,held,state,'same')
        for context in (dict(held,class_name='canned_juice'),dict(held,boot_id='old'),dict(held,run_id='other')):
            with self.assertRaises(ValueError):validate_destination_hold(previous,meta,context,state,'same')
        with self.assertRaises(ValueError):validate_destination_hold(previous,meta,held,dict(joints=[0.,0.,0.,-math.pi/2,0.,.65]),'same')

    def test_near_small_alignment_does_not_retreat_fifteen_cm_again(self):
        runtime=PepperRuntime.__new__(PepperRuntime)
        plan=runtime.approach_plan([.0009,-.006,.0001])
        self.assertEqual(plan['phase'],'small_open_finger_centering')
        self.assertEqual(plan['delta'],[0.,-.006,0.])
        self.assertEqual(runtime.approach_plan([.01,-.08,0.])['phase'],'clear_before_lateral_alignment')
        self.assertEqual(runtime.approach_plan([.15,-.006,0.])['phase'],'lateral_alignment')
    def test_measured_small_body_recovers_at_six_cm_without_changing_full_first_approach(self):
        r=PepperRuntime.__new__(PepperRuntime)
        r.accept_geometry(dict(class_name='ground_pepper',stamp_ns=1,body_radius_m=.0316),
                          dict(streams=dict(depth=dict(stamp_ns=1))))
        plan=r.approach_plan([.0165,-.023,.0062])
        self.assertEqual(plan['phase'],'clear_before_lateral_alignment')
        np.testing.assert_allclose(plan['delta'],[-.0435,0.,0.])
        self.assertEqual(r.approach_plan([.060,-.023,0.])['delta'],[0.,-.023,0.])
        self.assertEqual(r.approach_plan([.266,-.002,0.])['delta'],[.266,0.,0.])
        self.assertEqual(r.approach_plan([.006,-.008,0.])['phase'],'small_open_finger_centering')
    def test_compact_realign_scales_with_observed_radius_and_keeps_close_tolerance(self):
        r=PepperRuntime.__new__(PepperRuntime)
        r.accept_geometry(dict(class_name='ground_pepper',stamp_ns=1,body_radius_m=.043),
                          dict(streams=dict(depth=dict(stamp_ns=1))))
        self.assertAlmostEqual(r.approach_plan([.01,.025,0.])['delta'][0],-.058)
        r.contact_error=Mock(return_value=np.array([.01,.005,0.]))
        self.assertFalse(r.is_aligned([0,0,0],{}))
        r.contact_error.return_value=np.array([.01,.003,0.])
        self.assertTrue(r.is_aligned([0,0,0],{}))
    def test_other_adapter_does_not_enable_pepper_realign(self):
        from penguin_runtime import PenguinRuntime
        r=PenguinRuntime.__new__(PenguinRuntime)
        r.accept_geometry(dict(class_name='toy_penguin',stamp_ns=1,cap_radius_m=.023),dict(streams=dict(depth=dict(stamp_ns=1))))
        self.assertFalse(hasattr(r,'current_pepper_geometry'))
        self.assertAlmostEqual(r.approach_plan([.08,.006,0.])['delta'][0],-.07)
        with self.assertRaisesRegex(ValueError,'current_pepper_geometry'):
            PepperRuntime.__new__(PepperRuntime).accept_geometry(dict(class_name='ground_pepper',stamp_ns=2),
                dict(streams=dict(depth=dict(stamp_ns=1))))
    def test_continuation_requires_a_stopped_open_pose_and_no_previous_close(self):
        from pepper_transfer_automation import validate_open_pregrasp
        state=dict(joints=[.22,-1.7,0.,.1292036732,0.,.65])
        previous=dict(status='stopped_requires_actual_state_review',class_name='ground_pepper',stage='close_once',
            close_commands_sent=0,release_commands_sent=0,grasp_success_verified=False,last_fresh_robot_state=state)
        validate_open_pregrasp(previous,dict(boot_id='same'),state,'same')
        for item in (dict(previous,close_commands_sent=1),dict(previous,status='starting')):
            with self.assertRaisesRegex(ValueError,'zero_previous_close'):
                validate_open_pregrasp(item,dict(boot_id='same'),state,'same')
        with self.assertRaises(ValueError):validate_open_pregrasp(previous,dict(boot_id='old'),state,'same')
        held=dict(joints=state['joints'][:5]+[.4])
        with self.assertRaises(ValueError):validate_open_pregrasp(previous,dict(boot_id='same'),held,'same')

    def test_action_reuses_its_already_written_observation_without_duplicate_image_io(self):
        from canned_grasp_runtime import CanRuntime
        runtime=PepperRuntime.__new__(PepperRuntime);runtime.record=Mock()
        sample={};measurement={'stamp_ns':123}
        with tempfile.TemporaryDirectory() as d:
            output=Path(d);(output/'can-measurement.json').write_text('{}')
            with patch.object(CanRuntime,'save',return_value=output) as save:
                first=runtime.save(sample,'near',measurement)
                self.assertEqual(runtime.save(sample,'preclose',measurement),first)
                self.assertEqual(runtime.save(sample,'approach-00-before',measurement),first)
                self.assertEqual(save.call_count,1)
            self.assertTrue((output/'object-measurement.json').exists())

    def test_source_processing_checks_current_feedback_without_retimestamping_the_capture(self):
        from canned_carry_geometry import current_images_and_feedback
        _,_,s,_,_,_=fixture()
        now=s['streams']['depth']['stamp_ns']+180_000_000
        s['tracking_epoch']='pepper-source';s['joint_stamp_ns']=now-520_000_000
        names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        s['joints']=dict(zip(names,[0.,0.,0.,-math.pi/2,0.,.65]))
        state=dict(joints=[s['joints'][key] for key in names],stamp_ns=now-330_000_000)
        original=copy.deepcopy(s)
        runtime=PepperRuntime.__new__(PepperRuntime);runtime.epoch='pepper-source'
        runtime.state=Mock(return_value=state);runtime.node=Mock()
        runtime.node.get_clock.return_value.now.return_value.nanoseconds=now
        proof=runtime.fresh_measurement(s)
        self.assertTrue(proof['original_snapshot_unchanged']);self.assertEqual(s,original)
        state['stamp_ns']=now-650_000_000
        with self.assertRaisesRegex(ValueError,'fresh_current'):runtime.fresh_measurement(s)

    def test_current_head_pose_can_avoid_repeating_an_already_completed_motion(self):
        from pepper_transfer_automation import head_view_matches
        def snapshot(pan=0.,tilt=-.5):return dict(joints=dict(head_pan_joint=pan,head_tilt_joint=tilt))
        self.assertTrue(head_view_matches(snapshot(0.,-.4999999687)))
        self.assertFalse(head_view_matches(snapshot(0.,0.)))
        self.assertFalse(head_view_matches(snapshot(float('nan'))))
        self.assertFalse(head_view_matches(dict(joints={})))

    def test_delayed_tf_can_be_selected_without_relaxing_consumer_freshness(self):
        from canned_grasp_runtime import CanRuntime
        from synced_capture_selection import select_stamps
        now=10_000_000_000
        stamps=(now-250_000_000,now-240_000_000,now-150_000_000)
        history={key:{stamp:None} for key,stamp in zip(('rgb','depth','hand'),stamps)}
        def sample(runtime,timeout,processing_reserve_ns,*,joint_processing_reserve_ns=None,joint_max_age_ns=400_000_000):
            return select_stamps(history,now,ready=lambda _:True,
                joint_stamp_ns=now-350_000_000,processing_reserve_ns=processing_reserve_ns,
                joint_processing_reserve_ns=joint_processing_reserve_ns,joint_max_age_ns=joint_max_age_ns)
        runtime=PepperRuntime.__new__(PepperRuntime)
        with patch.object(CanRuntime,'sample',sample):
            self.assertIsNone(runtime.sample(processing_reserve_ns=200_000_000))
            self.assertIsNone(runtime.sample())
            self.assertEqual(runtime.sample(processing_reserve_ns=100_000_000),stamps)
            self.assertEqual(runtime.sample(processing_reserve_ns=0),stamps)
        too_old=tuple(now-600_000_000+i*10_000_000 for i in range(3))
        old_history={key:{stamp:None} for key,stamp in zip(('rgb','depth','hand'),too_old)}
        self.assertIsNone(select_stamps(old_history,now,ready=lambda _:True,
            joint_stamp_ns=now-100_000_000,processing_reserve_ns=100_000_000))

    def runtime(self):
        r=PepperRuntime.__new__(PepperRuntime)
        r.state=Mock(return_value=dict(joints=[0.,0.,0.,-math.pi/2,0.,.65]))
        r.move_joints=Mock(return_value=dict(status='passed'));return r
    def test_short_body_preparation_can_use_measured_lift_below_can_envelope(self):
        r=self.runtime();arm=r.prepare(.218,.0275)
        self.assertEqual(arm,[.218,-1.7,0.,.1292036732051034,0.])
        self.assertEqual(r.move_joints.call_count,2)
        self.assertLessEqual(r.vertical_alignment_tolerance_m,.006)
    def test_low_grasp_does_not_allow_fingers_through_the_table(self):
        r=self.runtime()
        with self.assertRaisesRegex(ValueError,'clearance'):r.prepare(.2,.02)
        r.move_joints.assert_not_called()
    def test_inactive_or_held_arm_is_not_treated_as_an_empty_tucked_start(self):
        r=self.runtime();r.state.return_value['joints'][5]=.4
        with self.assertRaisesRegex(ValueError,'empty_tucked'):r.prepare(.22,.0275)
        r.move_joints.assert_not_called()
    def test_default_and_grasp_only_flags_without_run_are_readonly(self):
        entry=Path(__file__).with_name('run_object_transfer.py')
        result=subprocess.run([sys.executable,str(entry),'--object','ground_pepper','--grasp-only'],capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr);self.assertFalse(json.loads(result.stdout)['robot_commands_sent'])
    def test_run_with_unreviewed_output_stops_before_ros_import_or_motion(self):
        entry=Path(__file__).with_name('run_object_transfer.py')
        result=subprocess.run([sys.executable,str(entry),'--object','ground_pepper','--run','--output','/tmp/never-pepper-live'],
            capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,2)
        self.assertIn('NEW D-drive output',result.stderr)

if __name__=='__main__':unittest.main()
