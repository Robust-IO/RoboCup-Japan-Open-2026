"""Functional success without exact target replay; failures are not hidden."""
import contextlib
import copy
import io
import math
from pathlib import Path
import tempfile
import unittest

import test_apple_transfer_automation as coordinator_fixture
import test_table_release_gate as release_fixture
import test_verify_table_placement as placement_fixture
from apple_transfer_auto_plan import placement_step
from apple_transfer_automation import run
from inverse_table_place_plan import build, release_from_current, cleanup_from_current
from task_outcome_policy import (hand_is_open, pose_matches, carry_start_from_current,
                                 arm_base_stability, carry_lower_from_current)
from test_verify_apple_lift_sequence import frame
from verify_apple_lift_sequence import pair
from verify_table_placement import verify


class FunctionalReleaseTests(unittest.TestCase):
    def setUp(self):
        self.f=release_fixture.ReleaseGateTest();self.f.setUp()
        self.addCleanup(self.f.tearDown)

    def test_safe_point_twenty_cm_from_plan_is_accepted_without_correction(self):
        f=self.f;f.measurement['center_base'][0]+=.20
        self.assertTrue(f.check()['ready'])
        result=placement_step(f.measurement,f.snapshot,f.plan,single_move=True)
        self.assertTrue(result['aligned']);self.assertEqual(result['delta_m'],[0.,0.,0.])
        self.assertGreater(result['distance_m'],.19)

    def test_unknown_or_outside_table_still_fails(self):
        f=self.f;f.measurement['center_base'][1]+=1.
        with self.assertRaisesRegex(ValueError,'observed_table_edge'):f.check()

    def test_current_arm_not_historical_posture_decides_release(self):
        f=self.f
        # Current image and joint feedback agree; actual object is over the table.
        f.joints[0]+=.02;f.joints[1]+=.05
        f.snapshot['joints']['arm_lift_joint']=f.joints[0]
        f.snapshot['joints']['arm_flex_joint']=f.joints[1]
        self.assertTrue(f.check()['ready'])

    def test_arm_motion_after_image_still_invalidates_release(self):
        self.f.kw['current_joints']=self.f.joints.copy()
        self.f.kw['current_joints'][1]+=.1
        with self.assertRaisesRegex(ValueError,'pose_changed'):self.f.check()

    def test_table_contact_does_not_require_artificial_five_mm_gap(self):
        self.f.measurement['center_base'][2]-=.02
        self.assertAlmostEqual(self.f.check()['estimated_bottom_gap_m'],0.)

    def test_below_table_or_large_drop_still_fails(self):
        for z in (.42,.55):
            self.f.measurement['center_base'][2]=z
            with self.subTest(z=z),self.assertRaisesRegex(ValueError,'height_not_verified'):self.f.check()

    def test_release_changes_hand_only_not_arm_to_expected_pose(self):
        current=[.23,-1.65,.02,.15,.01,.42]
        plan=release_from_current(build('release',.486,.415),current)
        self.assertEqual(plan['initial'],current)
        self.assertEqual(plan['steps'][0]['goal'][:5],current[:5])
        self.assertEqual(plan['steps'][0]['goal'][5],.65)
        self.assertEqual(current[5],.42)

    def test_release_rebasing_does_not_allow_arm_motion_plans_or_empty_hand(self):
        for stage in ('prepare','raise','retract'):
            with self.subTest(stage=stage),self.assertRaises(ValueError):
                release_from_current(build(stage,.486,.415),[.23,-1.65,0,.15,0,.42])
        with self.assertRaises(ValueError):
            release_from_current(build('release',.486,.415),[.23,-1.65,0,.15,0,.65])

    def test_cleanup_uses_actual_lift_and_opening_not_plan_values(self):
        current=[.23,-1.69,0,.14,0,.62]
        raised=cleanup_from_current(build('raise',.486,.415),'raise',current)
        self.assertAlmostEqual(raised['steps'][0]['goal'][0],.31)
        self.assertEqual(raised['steps'][0]['goal'][1:],current[1:])
        tucked=cleanup_from_current(build('retract',.486,.415),'retract',raised['steps'][-1]['goal'])
        self.assertEqual(tucked['initial'],raised['steps'][-1]['goal'])
        self.assertEqual(tucked['steps'][-1]['goal'],[0.,0.,0.,-math.pi/2,0.,.62])

    def test_cleanup_does_not_open_a_held_object_or_accept_wrong_arm_orientation(self):
        for current in ([.23,-1.7,0,.13,0,.42],[.23,0,0,.13,0,.65]):
            with self.assertRaises(ValueError):cleanup_from_current(build('raise',.486,.415),'raise',current)


class CarryBaseStabilityTests(unittest.TestCase):
    def test_recorded_two_degree_yaw_is_allowed_only_for_held_carry(self):
        anchor=[10.063109485896106,-6.638299542088921,-.07247560045414105]
        actual=[10.06327369051011,-6.636991196992159,-.03601652513277341]
        self.assertFalse(arm_base_stability(anchor,actual)['allowed'])
        decision=arm_base_stability(anchor,actual,carrying=True)
        self.assertTrue(decision['allowed'])
        self.assertAlmostEqual(decision['translation_m'],.0013186091337021887)
        self.assertAlmostEqual(decision['yaw_rad'],.036459075321367634)
        self.assertEqual(decision['actual'],actual)

    def test_carry_does_not_allow_large_translation_or_rotation(self):
        for pose in ([.016,0,0],[0,0,math.radians(5.1)]):
            with self.subTest(pose=pose):
                self.assertFalse(arm_base_stability([0,0,0],pose,carrying=True)['allowed'])

    def test_grasp_and_release_original_yaw_check_is_unchanged(self):
        self.assertTrue(arm_base_stability([0,0,0],[.014,0,.029])['allowed'])
        self.assertFalse(arm_base_stability([0,0,0],[0,0,.031])['allowed'])

    def test_yaw_wraparound_is_a_small_rotation(self):
        self.assertTrue(arm_base_stability([0,0,math.pi-.01],[0,0,-math.pi+.01])['allowed'])

    def test_nonfinite_and_implicit_opt_in_are_not_allowed(self):
        for pose in ([0,0,float('nan')],[0,float('inf'),0],[True,0,0],[0,0]):
            with self.subTest(pose=pose),self.assertRaises(ValueError):
                arm_base_stability([0,0,0],pose,carrying=True)
        for flag in (1,'true',None):
            with self.assertRaises(ValueError):arm_base_stability([0,0,0],[0,0,0],carrying=flag)

    def test_explicit_folded_carry_completion_preserves_actual_grip(self):
        actual=[.354957938,0.,0.,-math.pi/2,0.,.382301898]
        saved=actual.copy()
        self.assertEqual(carry_lower_from_current(actual,.382301898),saved)
        self.assertEqual(actual,saved)

    def test_fold_completion_never_accepts_extended_or_open_or_wrong_held_arm(self):
        for actual,opening in (([.35,-1.7,0,.1292,0,.38],.38),
                               ([.35,0,0,-math.pi/2,0,.65],.65),
                               ([.35,0,0,-math.pi/2,0,.0],.38),
                               ([.35,0,0,-math.pi/2,0,float('nan')],.38),
                               ([.5,0,0,-math.pi/2,0,.38],.38)):
            with self.subTest(actual=actual),self.assertRaises(ValueError):
                carry_lower_from_current(actual,opening)


class FunctionalGraspTests(unittest.TestCase):
    def test_verified_eight_cm_lift_can_proceed_to_carry_without_ten_cm_pose(self):
        expected=[.35,-1.7,0,.1292,0,.415]
        actual=[.33,-1.69,0,.14,0,.42]
        self.assertEqual(carry_start_from_current(expected,actual,.25),actual)
        for bad in ([.26,-1.7,0,.1292,0,.415],[.33,-1.7,0,.1292,0,.65]):
            with self.assertRaises(ValueError):carry_start_from_current(expected,bad,.25)
    def test_lift_need_not_equal_ten_cm_or_repeat_all_angles(self):
        before=frame(1_000_000_000,.25,.54,.65)
        for rise in (.06,.08,.12,.15):
            after=frame(9_000_000_000,.25+rise,.54+rise,.43)
            after[1]['joints']['wrist_flex_joint']+=.02
            with self.subTest(rise=rise):
                self.assertLess(pair(before,after,.415)['object_palm_follow_error_m'],1e-8)

    def test_empty_hand_or_object_left_on_table_is_not_grasp(self):
        before=frame(1_000_000_000,.25,.54,.65)
        for after in (frame(9_000_000_000,.35,.54,.415),frame(9_000_000_000,.35,.64,0.),
                      frame(9_000_000_000,.35,.64,.65)):
            with self.assertRaises(ValueError):pair(before,after,.415)

    def test_lift_must_clear_support_and_object_must_not_slip(self):
        before=frame(1_000_000_000,.25,.54,.65)
        for rise,z in ((.01,.55),(.10,.60),(.10,.69)):
            with self.subTest(rise=rise,z=z),self.assertRaises(ValueError):
                pair(before,frame(9_000_000_000,.25+rise,z,.415),.415)

    def test_invalid_geometry_cannot_be_accepted_by_nan_comparisons(self):
        before=frame(1_000_000_000,.25,.54,.65)
        after=frame(9_000_000_000,.35,float('nan'),.415)
        with self.assertRaises(ValueError):pair(before,after,.415)


class FunctionalPlacementTests(unittest.TestCase):
    def fixture(self):return placement_fixture.PlacementTest().edge_fixture(3.76)
    def check(self,args):
        a,p,c,d,r,plan=args
        return verify(a,p,c,d,r,placement_plan=plan)

    def test_placed_object_is_success_even_if_arm_not_tucked(self):
        args=self.fixture()
        for image in args[0]['frames']:
            image['joints'].update(arm_lift_joint=.2,arm_flex_joint=-1.7,hand_motor_joint=.62)
        result=self.check(args)
        self.assertTrue(result['placement_success_verified'])
        self.assertFalse(result['robot_ready_for_next_task'])
        self.assertTrue(result['cleanup_required'])

    def test_cleanup_failure_cannot_negate_observed_object_placement(self):
        args=self.fixture();args[2]['raise']['status']='failed';args[2].pop('retract')
        result=self.check(args)
        self.assertTrue(result['placement_success_verified'])
        self.assertFalse(result['cleanup_actions_completed'])

    def test_small_settling_is_not_exact_position_failure(self):
        args=self.fixture();args[0]['frames'][0]['apple_measurements'][0]['center_map'][0]+=.015
        self.assertTrue(self.check(args)['placement_success_verified'])

    def test_dropped_floating_moving_or_still_held_object_not_success(self):
        for mutation in ('edge','floating','moving','closed','ambiguous'):
            args=self.fixture();image=args[0]['frames'][0];obj=image['apple_measurements'][0]
            if mutation=='edge':obj['center_map'][1]=3.644
            elif mutation=='floating':obj['estimated_bottom_gap_m']=.08
            elif mutation=='moving':obj['center_map'][0]+=.04
            elif mutation=='closed':image['joints']['hand_motor_joint']=.42
            else:image['apple_measurements'].append(copy.deepcopy(obj))
            with self.subTest(mutation=mutation):self.assertFalse(self.check(args)['placement_success_verified'])


class CleanupCoordinatorTests(unittest.TestCase):
    class Backend(coordinator_fixture.FakeBackend):
        mode='live'
        def perform(self,stage):
            result=super().perform(stage)
            if stage=='withdraw':
                return dict(verified=False,evidence='SYNTHETIC_TEST_ONLY',cleanup_required=True,
                    deferred_to_object_verification=True,reason='arm_cleanup_incomplete')
            if stage=='verify_placement':return dict(result,robot_ready_for_next_task=False)
            return result

    def execute(self,b):
        with tempfile.TemporaryDirectory() as tmp,contextlib.redirect_stdout(io.StringIO()):
            return run(coordinator_fixture.CONFIG,'test-task',Path(tmp)/'new',b)

    def test_cleanup_failure_continues_to_read_only_object_proof_not_more_motion(self):
        b=self.Backend();result=self.execute(b)
        self.assertEqual(b.calls,list(coordinator_fixture.STAGES))
        self.assertTrue(result['task_objective_completed']);self.assertTrue(result['placement_verified'])
        self.assertFalse(result['robot_ready_for_next_task']);self.assertTrue(result['requires_fresh_state_audit'])
        self.assertEqual(result['status'],'automatic_transfer_verified_cleanup_required')
        self.assertEqual(b.calls.count('release'),1);self.assertFalse(result['task_finished_sent'])

    def test_no_success_if_read_only_placement_proof_fails(self):
        b=self.Backend(unverified='verify_placement');result=self.execute(b)
        self.assertFalse(result['task_objective_completed']);self.assertFalse(result['placement_verified'])
        self.assertEqual(result['status'],'stopped')

    def test_release_failure_does_not_gain_cleanup_exception(self):
        b=self.Backend(fail='release');result=self.execute(b)
        self.assertEqual(result['status'],'stopped');self.assertNotIn('withdraw',b.calls)

    def test_process_cleanup_error_does_not_erase_completed_physical_result(self):
        b=self.Backend()
        def fail_cleanup():raise RuntimeError('observation_process_cleanup_failed')
        b.close=fail_cleanup
        result=self.execute(b)
        self.assertTrue(result['placement_verified']);self.assertTrue(result['task_objective_completed'])
        self.assertFalse(result['robot_ready_for_next_task'])

    def test_joint_allowances_are_not_unbounded_or_nan_acceptance(self):
        expected=[.25,-1.7,0,.13,0,.415]
        self.assertTrue(pose_matches([.254,-1.69,0,.14,0,.42],expected,endpoint=True))
        self.assertFalse(pose_matches([.35,-1.7,0,.13,0,.415],expected))
        self.assertFalse(pose_matches([float('nan'),-1.7,0,.13,0,.415],expected))
        for value in (float('nan'),.415,False):self.assertFalse(hand_is_open(value))


if __name__=='__main__':unittest.main()
