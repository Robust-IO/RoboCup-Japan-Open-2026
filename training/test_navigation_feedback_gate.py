import unittest
from navigation_feedback_gate import validate_ages, acquire_fresh_feedback, NavigationFeedbackError


class FeedbackTest(unittest.TestCase):
    def test_explicit_static_arm_allowance_leaves_scan_and_tf_limits_unchanged(self):
        validate_ages(dict(joints=.824,scan=.2,tf=.2),joint_max_age_s=.85)
        with self.assertRaises(NavigationFeedbackError):validate_ages(dict(joints=.824,scan=.2,tf=.2))
        for key in ('scan','tf'):
            ages=dict(joints=.824,scan=.2,tf=.2);ages[key]=.701
            with self.assertRaises(NavigationFeedbackError):validate_ages(ages,joint_max_age_s=.85)
        with self.assertRaises(NavigationFeedbackError):validate_ages(dict(joints=1.001,scan=.2,tf=.2),joint_max_age_s=.85)
    def test_existing_age_bounds_unchanged(self):
        validate_ages(dict(joints=-.05,scan=.7,tf=.2))
    def test_missing_or_old_or_future_stream_identified(self):
        for key in ('joints','scan','tf'):
            for value in (None,float('nan'),.70001,-.05001):
                ages=dict(joints=.1,scan=.1,tf=.1);ages[key]=value
                with self.subTest(key=key,value=value),self.assertRaisesRegex(RuntimeError,':'+key+'$'):
                    validate_ages(ages)


class AcquisitionTest(unittest.TestCase):
    def test_new_joint_policy_keeps_the_single_150ms_reacquisition_deadline(self):
        def check():
            validate_ages(dict(joints=.86 if self.now<.05 else .55,scan=.2,tf=.2),joint_max_age_s=.85)
            return 'fresh'
        self.assertEqual(acquire_fresh_feedback(check,self.pump,clock=lambda:self.now,joint_max_age_s=.85),'fresh')
        self.assertLess(self.now,.15)
    def setUp(self):
        self.now=0.;self.events=[];self.pumps=0

    def pump(self,dt):
        self.pumps+=1;self.now+=dt

    def acquire(self,check,pump=None):
        return acquire_fresh_feedback(check,pump or self.pump,on_event=self.events.append,clock=lambda:self.now)

    def check_age(self,age):
        validate_ages(dict(joints=.1,scan=age,tf=.1));return 'fresh'

    def test_already_fresh_no_wait(self):
        self.assertEqual(self.acquire(lambda:self.check_age(.7)),'fresh')
        self.assertEqual((self.pumps,self.events),(0,[]))

    def test_short_upstream_gap_accepts_only_new_fresh_sample(self):
        def check():return self.check_age(.709+self.now if self.now<.08 else .002)
        self.assertEqual(self.acquire(check),'fresh')
        self.assertLess(self.now,.15)
        self.assertEqual([e['event'] for e in self.events],['waiting_for_new_sample','new_fresh_sample_received'])

    def test_no_new_message_cannot_pass_or_extend_budget(self):
        with self.assertRaisesRegex(RuntimeError,'^stale_navigation_feedback:scan$'):
            self.acquire(lambda:self.check_age(.701+self.now))
        self.assertLessEqual(self.now,.150001)

    def test_invalid_future_missing_or_already_very_old_fail_immediately(self):
        for age in (None,float('nan'),float('inf'),True,-.051,.851,2.):
            with self.subTest(age=age),self.assertRaises(NavigationFeedbackError):
                self.acquire(lambda:self.check_age(age))
        self.assertEqual(self.pumps,0)

    def test_hold_control_or_tf_exception_never_retried(self):
        for error in ('carry_pose_changed:hand_motor_joint','finite_base_controller_present','missing_feedback','TF lookup failed'):
            def check():raise RuntimeError(error)
            with self.subTest(error=error),self.assertRaisesRegex(RuntimeError,error):self.acquire(check)
        self.assertEqual(self.pumps,0)

    def test_pose_change_during_wait_propagates(self):
        def check():
            if not self.pumps:return self.check_age(.709)
            raise RuntimeError('carry_pose_changed:hand_motor_joint')
        with self.assertRaisesRegex(RuntimeError,'carry_pose_changed:hand_motor_joint'):self.acquire(check)

    def test_other_invalid_stream_cannot_hide_behind_small_overrun(self):
        for invalid in (None,float('nan'),-.051,.851):
            def check():validate_ages(dict(joints=.709,scan=.1,tf=invalid))
            with self.subTest(invalid=invalid),self.assertRaisesRegex(NavigationFeedbackError,':tf$'):
                self.acquire(check)
        self.assertEqual(self.pumps,0)

    def test_stream_switch_does_not_restart_deadline(self):
        def check():
            validate_ages(dict(joints=.1,scan=.71 if self.now<.09 else .01,tf=.01 if self.now<.09 else .71))
        with self.assertRaises(NavigationFeedbackError):self.acquire(check)
        self.assertLessEqual(self.now,.150001)

    def test_slow_pump_cannot_authorize_late_success(self):
        def check():return self.check_age(.709 if not self.pumps else .001)
        def slow(dt):self.pumps+=1;self.now+=.151
        with self.assertRaises(NavigationFeedbackError):self.acquire(check,slow)

    def test_slow_check_cannot_authorize_late_success(self):
        def check():
            if not self.pumps:return self.check_age(.709)
            self.now+=.151
            return self.check_age(.001)
        with self.assertRaises(NavigationFeedbackError):self.acquire(check)


if __name__=='__main__':unittest.main()
