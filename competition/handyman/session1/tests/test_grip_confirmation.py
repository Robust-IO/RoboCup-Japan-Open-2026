import unittest
from grasp_runtime.autograsp.grip_confirmation import RecloseEvidence,accepted
from grasp_runtime.autograsp.coordinator import LiftEvidence
from execution_plan import grasp_result

class GripConfirmationTests(unittest.TestCase):
    def test_fresh_stable_blocked_opening(self):
        g=RecloseEvidence()
        self.assertIsNone(g.observe(1_000_000_000,.42))
        self.assertIsNone(g.observe(1_100_000_000,.421))
        self.assertTrue(g.observe(1_200_000_000,.42))
    def test_stale_samples_do_not_confirm(self):
        g=RecloseEvidence()
        for _ in range(10):self.assertIsNone(g.observe(1_000_000_000,.42))
    def test_empty_closed_rejected(self):
        self.assertFalse(RecloseEvidence().observe(1_000_000_000,0.))
    def test_moving_opening_not_stable(self):
        g=RecloseEvidence()
        for i in range(4):self.assertIsNone(g.observe(1_000_000_000+i*100_000_000,.4-i*.02))
    def test_joint_policy_does_not_claim_visual_follow(self):
        e=vars(LiftEvidence(True,False,True,3,'lift_and_reclose_joint_feedback',.08,True))
        self.assertTrue(accepted(e));self.assertFalse(e['object_followed'])
        self.assertEqual(grasp_result(dict(status='grasp_verified',lift_evidence=e),target='canned_juice')['status'],'ok')
        e['lift_distance_m']=0.;self.assertFalse(accepted(e))
    def test_no_reclose_is_not_success(self):
        self.assertFalse(accepted(vars(LiftEvidence(True,False,True,3,'lift_and_reclose_joint_feedback',.08,False))))
