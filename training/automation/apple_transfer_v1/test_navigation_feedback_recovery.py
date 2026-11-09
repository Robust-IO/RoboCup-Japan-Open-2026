import unittest
from navigation_feedback_recovery import recover_stopped_feedback
from navigation_feedback_gate import NavigationFeedbackError

class RecoveryTests(unittest.TestCase):
    def setUp(self):self.now=0.;self.cancelled=0;self.events=[]
    def cancel(self):self.cancelled+=1
    def pump(self,dt):self.now+=dt
    def run_recovery(self,check):
        return recover_stopped_feedback(check,self.pump,self.cancel,on_event=self.events.append,clock=lambda:self.now,budget_s=.5)
    def test_cancel_first_then_require_two_new_valid_scan_tf_samples(self):
        def check():
            self.assertEqual(self.cancelled,1)
            if self.now<.1:raise NavigationFeedbackError('scan',.9)
            return dict(pose=[1,2,0],stamps_ns=dict(scan=int(self.now*1e9),tf=int(self.now*1e9)))
        result=self.run_recovery(check)
        self.assertGreater(self.now,.1);self.assertEqual(self.cancelled,1)
        self.assertEqual(self.events[-1]['event'],'stopped_feedback_recovered')
        self.assertGreater(result['stamps_ns']['scan'],self.events[-1]['first_stamps_ns']['scan'])
    def test_same_fresh_sample_cannot_be_counted_twice(self):
        with self.assertRaisesRegex(RuntimeError,'recovery_expired'):
            self.run_recovery(lambda:dict(stamps_ns=dict(scan=123,tf=456)))
        self.assertLessEqual(self.now,.500001)
    def test_permanent_gap_cannot_extend_deadline(self):
        def check():raise NavigationFeedbackError('scan',.9+self.now)
        with self.assertRaisesRegex(RuntimeError,'recovery_expired'):self.run_recovery(check)
        self.assertEqual(self.cancelled,1);self.assertLessEqual(self.now,.500001)
    def test_carry_change_and_invalid_future_feedback_are_not_retried(self):
        for error in (RuntimeError('carry_pose_changed:hand_motor_joint'),NavigationFeedbackError('tf',-.1)):
            self.now=0
            def check():raise error
            with self.assertRaises(type(error)):self.run_recovery(check)
            self.assertEqual(self.now,0.)
    def test_cancel_failure_prevents_check(self):
        def cancel():raise RuntimeError('navigation_cancel_not_confirmed')
        def check():self.fail('cannot use feedback before confirmed cancellation')
        with self.assertRaisesRegex(RuntimeError,'cancel_not_confirmed'):
            recover_stopped_feedback(check,self.pump,cancel)
    def test_late_check_cannot_authorize_resume(self):
        def check():
            self.now+=.6
            return dict(stamps_ns=dict(scan=123,tf=456))
        with self.assertRaisesRegex(RuntimeError,'recovery_expired'):self.run_recovery(check)

if __name__=='__main__':unittest.main()
