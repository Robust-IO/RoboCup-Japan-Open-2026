import unittest
from unittest.mock import patch
from types import SimpleNamespace
from grasp_runtime.autograsp.cyclic_recovery import CyclicCoordinator
from grasp_runtime.autograsp.coordinator import AttemptFailure,LiftEvidence
from grasp_runtime.autograsp.recovery import Failure
from grasp_runtime.autograsp.planner import Policy

class CyclicTests(unittest.TestCase):
    def backend(self,success_at=9,cancel_at=None):
        calls=[];b=SimpleNamespace(candidate={'id':'x','strategy':'overhead'},count=0,
            holding_state=lambda:'empty',stop=lambda:calls.append('stop'),
            observe_stopped=lambda t:None,reset_rotation_cycle=lambda:calls.append('reset'),
            begin_rotation_cycle=lambda:calls.append('begin'),rank_rotation_recovery=lambda d,f:d)
        def check():
            if cancel_at is not None and b.count>=cancel_at:raise RuntimeError('task_cancelled')
        b.check_task_cancelled=check
        def execute(c,s):
            calls.append('approach');raise AttemptFailure(Failure.MISALIGNED)
        b.execute_candidate=execute
        def rotate(n):
            b.count+=1;calls.append(n)
            if b.count<success_at:raise AttemptFailure(Failure.EMPTY_CLOSE)
        b.rotate_only_retry=rotate
        b.verify_lift=lambda:LiftEvidence(True,False,True,3,'lift_and_reclose_joint_feedback',.08,True)
        return b,calls
    def run_backend(self,b):
        with patch('grasp_runtime.autograsp.cyclic_recovery.select_template',return_value=(Policy(),{})),patch(
            'grasp_runtime.autograsp.cyclic_recovery.plan',return_value={'selected':b.candidate}):
            return CyclicCoordinator(b,None,Policy(),lambda e:None).run('dog_doll')
    def test_four_rotations_then_reset_and_no_five_attempt_limit(self):
        b,c=self.backend();r=self.run_backend(b)
        self.assertEqual(r['status'],'grasp_verified');self.assertEqual(r['corrections'],9)
        self.assertEqual(c,['approach','begin',1,2,3,4,'reset','approach','begin',1,2,3,4,'reset','approach','begin',1,'stop'])
    def test_success_ends_before_four(self):
        b,c=self.backend(success_at=2);self.run_backend(b)
        self.assertEqual(c,['approach','begin',1,2,'stop'])
    def test_unfinished_approach_rotates_before_reset(self):
        b,c=self.backend(success_at=1)
        b.contact_attempt_started=False
        def reset():
            c.append('reset');b.contact_attempt_started=True
        b.reset_rotation_cycle=reset
        self.run_backend(b)
        self.assertEqual(c,['approach','begin',1,'stop'])
    def test_cancellation_exits_unlimited_loop(self):
        b,c=self.backend(success_at=100,cancel_at=6)
        with self.assertRaisesRegex(RuntimeError,'task_cancelled'):self.run_backend(b)
        self.assertEqual(c[-1],'stop');self.assertEqual(b.count,6)
    def test_side_posture_retains_rotation_request(self):
        from grasp_runtime.autograsp.kinematics import posture
        self.assertEqual(posture('side',.5)[4],.5)
    def test_first_plan_neutral_then_recovery_allows_rotation(self):
        b,c=self.backend(success_at=5);policies=[]
        def planner(scene,hand,policy):
            policies.append(policy);return {'selected':b.candidate}
        with patch('grasp_runtime.autograsp.cyclic_recovery.select_template',return_value=(Policy(),{})),patch('grasp_runtime.autograsp.cyclic_recovery.plan',side_effect=planner):
            CyclicCoordinator(b,None,Policy(),lambda e:None).run('dog_doll')
        self.assertTrue(policies[0].neutral_wrist_only)
        self.assertFalse(policies[0].rotation_search)
        self.assertFalse(policies[1].neutral_wrist_only)
        self.assertTrue(policies[1].rotation_search)
    def test_neutral_generator_has_no_rotated_candidates(self):
        import numpy as np
        from grasp_runtime.autograsp.planner import generate,Scene
        x,y,z=np.meshgrid(np.linspace(.4,.47,12),np.linspace(-.025,.025,12),np.linspace(.42,.48,8))
        rows=generate(Scene(np.c_[x.ravel(),y.ravel(),z.ravel()],np.empty((0,3)),np.array([0,0,.4]),1),Policy(neutral_wrist_only=True,rotation_search=True))
        self.assertTrue(rows)
        self.assertTrue(all(row['angle']==0 for row in rows))

    def test_low_wide_no_side_candidate_folds_before_overhead(self):
        b,c=self.backend();policies=[]
        b.execute_candidate=lambda candidate,scene:c.append('execute-overhead')
        def planner(scene,hand,policy):
            policies.append(policy)
            if policy.preferred_strategy=='side':return {'selected':None}
            self.assertIn('reset',c)
            return {'selected':b.candidate}
        with patch('grasp_runtime.autograsp.cyclic_recovery.select_template',return_value=(Policy(preferred_strategy='side',allowed_strategies=('side',)),{'overhead_after_fold':True})),patch('grasp_runtime.autograsp.cyclic_recovery.plan',side_effect=planner):
            result=CyclicCoordinator(b,None,Policy(),lambda e:None).run('dog_doll')
        self.assertEqual(policies[0].allowed_strategies,('side',))
        self.assertEqual(policies[1].allowed_strategies,('side',))
        self.assertEqual(policies[2].allowed_strategies,('overhead',))
        self.assertEqual(result['cycles'],2)
        self.assertEqual(c,['reset','execute-overhead','stop'])

    def test_unchanged_decision_filtered_once(self):
        b,c=self.backend(success_at=1);filtered=[]
        def filtering(decision):
            filtered.append(decision);return decision
        b.filter_approach_decision=filtering
        self.run_backend(b)
        self.assertEqual(len(filtered),1)

    def test_reranked_decision_is_filtered_again(self):
        b,c=self.backend(success_at=1);filtered=[]
        b.resume_failed_pose={'id':'old'}
        def filtering(decision):
            filtered.append(decision);return decision
        b.filter_approach_decision=filtering
        self.run_backend(b)
        self.assertEqual(len(filtered),2)

    def test_trial_close_before_wrist_rotation(self):
        b,c=self.backend(success_at=1)
        def execute(*args):raise AttemptFailure(Failure.MISALIGNED,'contact_band_unobserved')
        b.execute_candidate=execute
        b.try_aligned_trial_before_rotation=lambda reason:c.append('trial') or True
        result=self.run_backend(b)
        self.assertEqual(result['status'],'grasp_verified')
        self.assertEqual(c,['trial','stop'])

    def test_failed_trial_continues_rotation(self):
        b,c=self.backend(success_at=1)
        def execute(*args):raise AttemptFailure(Failure.MISALIGNED,'contact_band_unobserved')
        def trial(reason):
            c.append('trial');raise AttemptFailure(Failure.EMPTY_CLOSE)
        b.execute_candidate=execute;b.try_aligned_trial_before_rotation=trial
        self.run_backend(b)
        self.assertEqual(c,['trial','begin',1,'stop'])

    def test_width_failure_does_not_enable_trial(self):
        b,c=self.backend(success_at=1)
        def execute(*args):raise AttemptFailure(Failure.MISALIGNED,'material_outside_actual_open_gap')
        b.execute_candidate=execute
        b.try_aligned_trial_before_rotation=lambda reason:self.fail('width must not be waived')
        self.run_backend(b)
