import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

from canned_carry_geometry import current_held,carry_initial,tucked_holding,compare_held,current_images_and_feedback
from test_upright_cylinder_geometry import rendered_cylinder


class CanCarryTests(unittest.TestCase):
    def setUp(self):
        rgb,self.depth,self.snapshot,d,self.center=rendered_cylinder()
        self.snapshot['palm_base_at_depth']=(self.center-[.066,0.,.001]).tolist()
        self.snapshot['joints']=dict(hand_motor_joint=.48)
        self.reference=dict(can_grasp_and_lift_verified=True,held_opening_rad=.48,
            held_center_minus_palm_base_m=[.066,0.,.001],can_footprint_radius_m=.025,can_height_m=.11)
    def measure(self):return current_held(self.depth,self.snapshot,self.reference)
    def test_current_can_is_measured_not_inferred_from_grip(self):
        result=self.measure()
        np.testing.assert_allclose(result['measurement']['center_m'],self.center,atol=.003)
        self.assertTrue(result['current_depth_surface_verified']);self.assertFalse(result['motion_authorized'])
    def test_grip_with_no_depth_cannot_prove_held(self):
        self.depth*=0
        with self.assertRaises(ValueError):self.measure()
    def test_open_or_drifting_grip_is_not_carried(self):
        for opening in (.65,.0,.54):
            self.snapshot['joints']['hand_motor_joint']=opening
            with self.subTest(opening=opening),self.assertRaises(ValueError):self.measure()
    def test_unverified_predecessor_cannot_enable_carry(self):
        self.reference['can_grasp_and_lift_verified']=False
        with self.assertRaises(ValueError):self.measure()
    def test_old_offset_cannot_fabricate_another_surface(self):
        self.reference['held_center_minus_palm_base_m'][0]+=.15
        with self.assertRaises(ValueError):self.measure()
    def test_actual_lifted_start_not_exactly_ten_cm_is_accepted(self):
        actual=[.36,-1.7,0.,.1292036732051034,0.,.48]
        self.assertEqual(carry_initial(actual,.281,.48),actual)
    def test_not_lifted_or_opened_start_is_rejected(self):
        for lift,opening in ((.30,.48),(.38,.65)):
            with self.assertRaises(ValueError):carry_initial([lift,-1.7,0.,.1292036732051034,0.,opening],.281,.48)
    def test_tucked_holding_accepts_functional_pose_not_grasp_pose(self):
        self.assertTrue(tucked_holding([0.,0.,0.,-math.pi/2,0.,.48],.48))
        self.assertFalse(tucked_holding([.38,-1.7,0.,.1292,0.,.48],.48))
        self.assertFalse(tucked_holding([0.,0.,0.,-math.pi/2,0.,.65],.48))
    def test_same_measured_relative_geometry_passes(self):
        result=self.measure();proof=compare_held(result,copy.deepcopy(result))
        self.assertTrue(proof['can_still_held_verified']);self.assertFalse(proof['placement_verified'])
    def test_slip_or_missing_geometry_fails_comparison(self):
        before=self.measure()
        for kind in ('slip','radius','depth'):
            after=copy.deepcopy(before)
            if kind=='slip':after['center_minus_palm_base_m'][0]+=.03
            if kind=='radius':after['measurement']['footprint_radius_m']+=.01
            if kind=='depth':after['current_depth_surface_verified']=False
            with self.subTest(kind=kind),self.assertRaises(ValueError):compare_held(before,after)
    def test_default_entry_no_ros_no_motion_no_release(self):
        script=Path(__file__).with_name('run_canned_carry_trial.py')
        result=subprocess.run([sys.executable,str(script),'--navigate-destination'],capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr);plan=json.loads(result.stdout)
        self.assertFalse(plan['robot_commands_sent']);self.assertFalse(plan['grip_commands_sent'])
        self.assertFalse(plan['placement_supported']);self.assertIn('original_bedroom_to_living_route',plan['stages'])
    def test_grasp_capture_default_budget_is_unchanged(self):
        import inspect
        from canned_grasp_runtime import CanRuntime
        self.assertEqual(inspect.signature(CanRuntime.sample).parameters['processing_reserve_ns'].default,200_000_000)
    def test_no_model_budget_does_not_relax_final_feedback_age(self):
        from canned_grasp_geometry import fresh
        from test_canned_grasp_geometry import snapshot
        s=snapshot()
        fresh(s,1_380_000_000,'can-test')
        with self.assertRaises(ValueError):fresh(s,1_410_000_000,'can-test')
    def feedback_fixture(self):
        from test_canned_grasp_geometry import snapshot
        s=snapshot(1_900_000_000);s['joint_stamp_ns']=1_500_000_000
        s['joints']=dict(zip(('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint'),
            [.38,-1.7,0.,.1292,0.,.48]))
        state=dict(stamp_ns=1_650_000_000,joints=[.38,-1.7,0.,.1292,0.,.48])
        return s,state
    def test_fresh_current_feedback_does_not_retimestamp_old_snapshot(self):
        s,state=self.feedback_fixture();old=copy.deepcopy(s)
        proof=current_images_and_feedback(s,state,2_000_000_000,'can-test')
        self.assertEqual(s,old);self.assertEqual(proof['captured_joint_stamp_ns'],1_500_000_000)
        self.assertEqual(proof['current_joint_stamp_ns'],1_650_000_000)
        self.assertFalse(proof['captured_joint_sample_used_as_fresh_feedback'])
    def test_new_feedback_does_not_excuse_stale_images_or_joint_data(self):
        for kind in ('image','joint','sync','epoch'):
            s,state=self.feedback_fixture();epoch='can-test'
            if kind=='image':s['streams']['rgb']['stamp_ns']=1_500_000_000
            if kind=='joint':state['stamp_ns']=1_500_000_000
            if kind=='sync':s['streams']['rgb']['stamp_ns']-=40_000_000
            if kind=='epoch':epoch='other'
            with self.subTest(kind=kind),self.assertRaises(ValueError):current_images_and_feedback(s,state,2_000_000_000,epoch)
    def test_new_feedback_must_match_actual_image_arm_pose(self):
        s,state=self.feedback_fixture();state['joints'][0]+=.04
        with self.assertRaisesRegex(ValueError,'arm_changed'):
            current_images_and_feedback(s,state,2_000_000_000,'can-test')


if __name__=='__main__':unittest.main()
