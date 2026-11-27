import copy,unittest
from outcome_transfer_policy import load,stationary_action_feedback
from stop_observe_act import ObservationCycle
import test_outcome_transfer_policy as fixtures

class ObservationCycleTest(unittest.TestCase):
    def state(self,stamp=1_000_000_000):
        return dict(joints=[.25,-1.7,0,.129,0,.65],head=[0.,-.5],poses=dict(odom=[0.,0.,0.]),stamp_ns=stamp)
    def stopped(self):
        cycle=ObservationCycle(load()['feedback'])
        self.assertFalse(cycle.observe(self.state(),now=0.))
        self.assertTrue(cycle.observe(self.state(1_210_000_000),now=.21))
        return cycle
    def test_repeated_same_feedback_does_not_prove_stop(self):
        cycle=ObservationCycle(load()['feedback'])
        self.assertFalse(cycle.observe(self.state(),now=0.))
        self.assertFalse(cycle.observe(self.state(),now=.5))
    def test_measured_movement_restarts_settling_window(self):
        cycle=ObservationCycle(load()['feedback']);cycle.observe(self.state(),now=0.)
        moved=self.state(1_300_000_000);moved['poses']['odom'][0]=.03
        self.assertFalse(cycle.observe(moved,now=.3))
        stopped=copy.deepcopy(moved);stopped['stamp_ns']+=210_000_000
        self.assertTrue(cycle.observe(stopped,now=.51))
    def test_completed_command_still_requires_stopping_then_new_image(self):
        cycle=self.stopped();cycle.invalidate(now=.3,duration=2.)
        self.assertFalse(cycle.observe(self.state(3_000_000_000),now=2.))
        self.assertFalse(cycle.observe(self.state(3_400_000_000),now=2.4))
        self.assertTrue(cycle.observe(self.state(3_610_000_000),now=2.61))
        with self.assertRaisesRegex(ValueError,'new_image_after_stop'):
            cycle.capture_marker(dict(streams={k:dict(stamp_ns=3_000_000_000) for k in ('rgb','depth','hand')}))
        proof=cycle.capture_marker(dict(streams={k:dict(stamp_ns=3_620_000_000) for k in ('rgb','depth','hand')}))
        self.assertEqual(proof['generation'],1)
    def test_staying_stopped_does_not_repeat_settling_wait_for_every_frame(self):
        cycle=self.stopped();self.assertTrue(cycle.observe(self.state(1_220_000_000),now=.22))
    def test_long_computation_has_no_fixed_image_expiry_when_state_is_unchanged(self):
        fixture=fixtures.OutcomePolicyTest();s,state,now=fixture.feedback_scene(age=30.)
        proof=stationary_action_feedback(s,state,now,'current',current_generation=0)
        self.assertEqual(proof['oldest_image_age_s'],30.);self.assertFalse(proof['computation_age_is_rejection_gate'])
    def test_moving_and_returning_to_same_pose_still_invalidates_old_image(self):
        s,state,now=fixtures.OutcomePolicyTest().feedback_scene(age=2.)
        with self.assertRaisesRegex(ValueError,'motion_invalidated_observation'):
            stationary_action_feedback(s,state,now,'current',current_generation=1)
    def test_no_stopped_capture_proof_means_no_action(self):
        s,state,now=fixtures.OutcomePolicyTest().feedback_scene();s.pop('stop_observe_act')
        with self.assertRaisesRegex(ValueError,'stopped_capture_proof'):
            stationary_action_feedback(s,state,now,'current',current_generation=0)
    def test_local_command_construction_invalidates_capture_before_publish(self):
        from outcome_transfer_runtime import OutcomeRuntime
        class Base:
            @staticmethod
            def trajectory(*args):return args
        class Runtime(OutcomeRuntime,Base):pass
        runtime=Runtime();cycle=runtime.observation_cycle();self.assertEqual(cycle.generation,0)
        runtime.trajectory(['head_pan_joint'],[0.],2.)
        self.assertEqual(cycle.generation,1);self.assertFalse(cycle.stopped)
    def test_external_navigation_or_adjustment_invalidates_before_and_after(self):
        from outcome_transfer_runtime import OutcomeBackend
        from unittest.mock import Mock,patch
        backend=OutcomeBackend.__new__(OutcomeBackend);backend.observation_runtime=Mock()
        with patch('apple_transfer_auto_backend.LiveBackend.command',return_value=dict(status='passed')):
            self.assertEqual(backend.command([],None)['status'],'passed')
        self.assertEqual(backend.observation_runtime.invalidate_observations.call_count,2)
    def test_capture_waits_for_initial_tf_instead_of_failing_before_listener_warms(self):
        from outcome_transfer_runtime import OutcomeRuntime
        class Base:
            def sample(self,**kwargs):
                stamp=self.observation_cycle().stop_stamp_ns+1
                return dict(snapshot=dict(streams={k:dict(stamp_ns=stamp) for k in ('rgb','depth','hand')}))
        fixture=self
        class Runtime(OutcomeRuntime,Base):
            calls=0
            def observation_state(self):
                self.calls+=1
                if self.calls==1:raise LookupError('base_footprint does not exist yet')
                return fixture.state(1_000_000_000+self.calls*250_000_000)
            def pump(self,*_):pass
            def fresh_measurement(self,*_):return {}
        runtime=Runtime();sample=runtime.sample()
        self.assertTrue(sample['snapshot']['stop_observe_act']['captured_after_stop']);self.assertGreaterEqual(runtime.calls,3)

if __name__=='__main__':unittest.main()
