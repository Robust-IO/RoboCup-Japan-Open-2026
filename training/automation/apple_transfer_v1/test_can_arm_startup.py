import unittest
from can_arm_startup import CurrentTransformReadiness


class CanArmStartupTests(unittest.TestCase):
    def test_new_joint_ahead_of_tf_does_not_reset_a_fresh_coherent_sample(self):
        gate=CurrentTransformReadiness();observed={1_000_000_000}
        self.assertTrue(gate.ready(1_000_000_000,1_050_000_000,observed.__contains__))
        self.assertTrue(gate.ready(1_100_000_000,1_120_000_000,observed.__contains__))
        self.assertTrue(gate.ready(1_200_000_000,1_210_000_000,observed.__contains__))
        self.assertEqual(gate.stamp_ns,1_000_000_000)
    def test_old_selected_transform_expires_without_widening400ms(self):
        gate=CurrentTransformReadiness();available=lambda stamp:stamp==1_000_000_000
        self.assertTrue(gate.ready(1_000_000_000,1_000_000_000,available))
        self.assertFalse(gate.ready(1_450_000_000,1_450_000_000,available))
        self.assertIsNone(gate.stamp_ns)
    def test_missing_transform_never_fabricates_readiness(self):
        gate=CurrentTransformReadiness()
        self.assertFalse(gate.ready(1_000_000_000,1_050_000_000,lambda stamp:False))
    def test_missing_latest_feedback_invalidates_even_an_existing_transform(self):
        for latest,now in [(None,1_000_000_000),(1_000_000_000,1_500_000_000),
                (1_050_000_000,1_000_000_000),(True,1_000_000_000)]:
            gate=CurrentTransformReadiness();gate.stamp_ns=1_000_000_000
            self.assertFalse(gate.ready(latest,now,lambda stamp:True))
            self.assertIsNone(gate.stamp_ns)
    def test_tf_disappearance_and_clock_reset_require_a_new_actual_transform(self):
        gate=CurrentTransformReadiness();self.assertTrue(gate.ready(2_000_000_000,2_000_000_000,lambda stamp:True))
        self.assertFalse(gate.ready(2_100_000_000,2_100_000_000,lambda stamp:False))
        self.assertFalse(gate.ready(1_000_000_000,1_000_000_000,lambda stamp:stamp==2_000_000_000))
        self.assertTrue(gate.ready(1_000_000_000,1_000_000_000,lambda stamp:stamp==1_000_000_000))
    def test_repeated_asynchronous_joint_tf_arrivals_complete_original_stable_wait(self):
        from transfer_efficiency import wait_ready
        now=[1.];gate=CurrentTransformReadiness()
        def check():
            # Every100ms the new joint arrives first; its TF appears30ms later.
            latest=1_000_000_000+round((now[0]-1.)//.1)*100_000_000
            current=round(now[0]*1e9)
            return gate.ready(latest,current,lambda stamp:current>=stamp+30_000_000)
        def pump(seconds):now[0]+=seconds
        wait_ready(check,pump,timeout=1.,stable_s=.15,clock=lambda:now[0])
        self.assertLess(now[0],1.4)


if __name__=='__main__':unittest.main()
