import ast
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from motion_speed import (NAV_SPEED_NAMES, PROFILE_PATH, NavigationSpeedLease,
                          arm_duration, duration_fields, finite_duration,
                          load_profile, navigation_caps)


class TimingTest(unittest.TestCase):
    def setUp(self):
        self.fast = load_profile('balanced_v1')
        self.old = load_profile('baseline')

    def test_legacy_timing_and_no_navigation_write(self):
        self.assertEqual(arm_duration(self.old,'arm',[0]*6,[1]*6),6)
        self.assertEqual(arm_duration(self.old,'arm',[0]*6,[1]*6,segmented=False),4)
        self.assertEqual(finite_duration(self.old,[.32,0,0]),4)
        self.assertIsNone(navigation_caps(self.old,10))

    def test_gripper_closing_opening_are_not_accelerated(self):
        for holding in (False, True):
            self.assertEqual(arm_duration(self.fast,'hand',[0]*6,[0,0,0,0,0,.65],holding=holding),6)

    def test_lift_uses_actual_distance_and_stays_below_unity_limit(self):
        for height in (.04,.10,.25,.443):
            goal = [height,0,0,0,0,0]
            duration = arm_duration(self.fast,'arm',[0]*6,goal,holding=True)
            self.assertLessEqual(height/duration,.10)
            self.assertLess(duration,6.)
        self.assertEqual(arm_duration(self.fast,'arm',[0]*6,[.04,0,0,0,0,0]),1.2)

    def test_extension_and_retraction_keep_full_joint_travel(self):
        start=[.25,0,0,-math.pi/2,0,.395]
        goal=[.25,-1.7,0,.1292036732051034,0,.395]
        duration=arm_duration(self.fast,'arm',start,goal,holding=True)
        self.assertGreaterEqual(duration,1.7/.5)
        self.assertLess(duration,6)

    def test_joint_and_nanosecond_durations_are_not_truncated(self):
        self.assertEqual(duration_fields(1.2),(1,200000000))
        self.assertEqual(duration_fields(2.85),(2,850000000))
        with self.assertRaises(ValueError):duration_fields(float('nan'))

    def test_finite_movement_respects_existing_bounds_and_real_speed_limit(self):
        for delta in ([.32,0,0],[.25,.2,0],[.02,0,0],[0,0,.35]):
            duration=finite_duration(self.fast,delta,holding=True)
            self.assertLessEqual(math.hypot(*delta[:2])/duration,.1)
            self.assertLessEqual(abs(delta[2])/duration,.3)
        self.assertLess(finite_duration(self.fast,[.02,0,0]),finite_duration(self.fast,[.32,0,0]))
        with self.assertRaises(ValueError):finite_duration(self.fast,[.41,0,0])

    def test_near_target_finite_motion_is_slower_than_transit(self):
        duration=finite_duration(self.fast,[.08,0,0])
        self.assertLessEqual(.08/duration,.06)

    def test_explicit_long_single_placement_has_distance_based_duration(self):
        duration=finite_duration(self.fast,[.473,-.028,0],holding=True,max_translation_m=.6)
        self.assertGreaterEqual(duration,math.hypot(.473,.028)/.1)
        with self.assertRaises(ValueError):finite_duration(self.fast,[.601,0,0],holding=True,max_translation_m=.6)
        with self.assertRaises(ValueError):finite_duration(self.fast,[.473,0,0],max_translation_m=1.)

    def test_unlimited_single_placement_duration_is_distance_based_even_past_sixty_seconds(self):
        for profile in (self.fast,self.old):
            for distance in (.628,1.2,8.):
                duration=finite_duration(profile,[distance,0,0],holding=True,max_translation_m=None)
                self.assertGreaterEqual(duration,distance/.1)
                seconds,nanos=duration_fields(duration)
                self.assertAlmostEqual(seconds+nanos/1e9,duration)
        self.assertEqual(duration_fields(80.15),(80,150000000))

    def test_no_limit_does_not_accept_invalid_or_unrepresentable_trajectory(self):
        for distance in (float('nan'),float('inf'),1e308):
            with self.subTest(distance=distance),self.assertRaises(ValueError):
                finite_duration(self.fast,[distance,0,0],holding=True,max_translation_m=None)
        for seconds in (0.,float('inf'),2**31):
            with self.subTest(seconds=seconds),self.assertRaises(ValueError):duration_fields(seconds)

    def test_navigation_payload_and_short_leg_caps(self):
        self.assertEqual(navigation_caps(self.fast,5),[.28,.28,.4])
        self.assertEqual(navigation_caps(self.fast,5,holding=True),[.24,.24,.3])
        self.assertEqual(navigation_caps(self.fast,1.4),[.2,.2,.2])
        self.assertEqual(navigation_caps(self.fast,1.4,holding=True),[.2,.2,.2])

    def test_user_fast_navigation_changes_only_transit_caps(self):
        fast=load_profile('carry_fast_v1')
        for holding in (False,True):
            self.assertEqual(navigation_caps(fast,5,holding=holding),[.28,.28,.8])
            self.assertEqual(navigation_caps(fast,1.4,holding=holding),[.2,.2,.8])
        self.assertEqual(fast['arm'],self.fast['arm'])
        self.assertEqual(fast['finite_base'],self.fast['finite_base'])

    def test_fast_navigation_envelope_does_not_relax_old_profile(self):
        document=json.loads(PROFILE_PATH.read_text())
        for profile,key,value in [('balanced_v1','holding_angular_rps',.8),('carry_fast_v1','holding_linear_mps',.31),('carry_fast_v1','empty_angular_rps',1.2)]:
            bad=copy.deepcopy(document);bad['profiles'][profile]['navigation'][key]=value
            with tempfile.TemporaryDirectory() as root:
                path=Path(root)/'p.json';path.write_text(json.dumps(bad))
                with self.subTest(profile=profile,key=key),self.assertRaises(ValueError):load_profile(profile,path)

    def test_tuning_cannot_exceed_reviewed_envelope_or_unity_limit(self):
        document=json.loads(PROFILE_PATH.read_text())
        for section,key,value in [('navigation','empty_linear_mps',.31),('arm','holding_lift_mps',.16),
                                  ('arm','empty_angular_rps',1.1),('finite_base','empty_linear_mps',float('nan'))]:
            bad=copy.deepcopy(document);bad['profiles']['balanced_v1'][section][key]=value
            with tempfile.TemporaryDirectory() as root:
                path=Path(root)/'profile.json';path.write_text(json.dumps(bad))
                with self.subTest(key=key),self.assertRaises(ValueError):load_profile('balanced_v1',path)

    def test_profile_is_propagated_through_every_command_chain(self):
        root=Path(__file__).parent
        for name in ('pregrasp_raised_trial.py','reviewed_carry_adjustment.py','navigate_carry_trial.py',
                     'run_bedroom_apple_regrasp.py','run_apple_close_lift_trial.py','apple_transfer_auto_backend.py'):
            tree=ast.parse((root/name).read_text())
            strings=[n.value for n in ast.walk(tree) if isinstance(n,ast.Constant) and isinstance(n.value,str)]
            with self.subTest(name=name):self.assertIn('--motion-profile',strings)


class LeaseTest(unittest.TestCase):
    def setUp(self):
        self.state=[.2,.2,.2];self.writes=[]
        def write(values):self.writes.append(list(values));self.state[:]=values
        self.lease=NavigationSpeedLease(lambda:self.state[:],write)

    def test_only_three_speed_parameters_are_in_scope(self):
        self.assertEqual(len(NAV_SPEED_NAMES),3)
        self.assertTrue(all(name.startswith('FollowPath.max_') for name in NAV_SPEED_NAMES))

    def test_speed_up_then_short_leg_then_restore_original(self):
        self.lease.apply([.28,.28,.4]);self.lease.apply([.2,.2,.2]);self.lease.restore()
        self.assertEqual(self.state,[.2,.2,.2]);self.assertTrue(self.lease.restored)

    def test_exact_preexisting_custom_speed_is_restored(self):
        self.state[:]=[.18,.18,.17]
        self.lease.apply([.24,.24,.3]);self.lease.restore()
        self.assertEqual(self.state,[.18,.18,.17])

    def test_baseline_performs_no_read_or_write(self):
        def fail(*args):raise AssertionError('baseline touched parameters')
        lease=NavigationSpeedLease(fail,fail);lease.apply(None);lease.restore()
        self.assertFalse(lease.changed);self.assertTrue(lease.restored)

    def test_uncertain_atomic_timeout_still_restores_not_retries(self):
        original_write=self.lease.write
        def ambiguous(values):
            original_write(values)
            if values==[.28,.28,.4]:raise TimeoutError('ambiguous response')
        self.lease.write=ambiguous
        with self.assertRaises(TimeoutError):self.lease.apply([.28,.28,.4])
        self.lease.restore()
        self.assertEqual(self.writes,[[.28,.28,.4],[.2,.2,.2]])
        self.assertTrue(self.lease.restored)

    def test_missing_readback_or_failed_restore_is_not_success(self):
        self.lease.apply([.28,.28,.4]);self.lease.read=lambda:[]
        with self.assertRaises(ValueError):self.lease.restore()
        self.assertFalse(self.lease.restored)


if __name__=='__main__':unittest.main()
