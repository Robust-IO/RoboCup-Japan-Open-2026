import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_recovery_plan import SearchRecoveryPlan


class RecoveryPlanTests(unittest.TestCase):
    def setUp(self):self.p=SearchRecoveryPlan('t','map',['v0','v1','v2'],100)
    def record(self,point='v0',outcome='observed_empty',now=1,**kw):
        return self.p.record(task='t',map_digest='map',point_id=point,outcome=outcome,
            now=now,cleanup_verified=kw.get('cleanup',True),observation_verified=kw.get('observed',True))
    def test_completed_point_not_repeated(self):
        self.record();r=self.p.propose(2)
        self.assertEqual(r['point_id'],'v1');self.assertEqual(r['completed_points'],['v0'])
    def test_weak_candidate_is_pending_not_negative(self):
        r=self.record(outcome='weak_candidate')
        self.assertEqual(r['point_id'],'v1');self.assertEqual(r['completed_points'],[])
        self.assertEqual(r['pending_review'][0]['point_id'],'v0')
        self.assertFalse(r['does_not_exist_authorized'])
    def test_motion_waits_for_both_checks(self):
        self.record(outcome='transient_motion')
        self.assertEqual(self.p.propose(2,sensors_ready=True)['proposal'],'none')
        self.assertEqual(self.p.propose(3,stationary=True)['proposal'],'none')
        r=self.p.propose(4,sensors_ready=True,stationary=True)
        self.assertEqual(r['point_id'],'v0');self.assertEqual(r['proposal'],'observe_current_point')
    def test_retry_does_not_extend_deadline(self):
        self.record(outcome='transient_motion',now=99)
        r=self.p.propose(100,sensors_ready=True,stationary=True)
        self.assertEqual(r['state'],'expired');self.assertEqual(r['deadline'],100)
    def test_second_motion_blocks(self):
        self.record(outcome='transient_motion');self.p.propose(2,sensors_ready=True,stationary=True)
        self.assertEqual(self.record(outcome='transient_motion',now=3)['state'],'blocked')
    def test_unverified_cleanup_blocks(self):
        self.assertEqual(self.record(cleanup=False)['state'],'blocked')
    def test_unverified_observation_blocks(self):
        self.assertEqual(self.record(observed=False,outcome='weak_candidate')['state'],'blocked')
    def test_wrong_map_rejected(self):
        with self.assertRaises(ValueError):
            self.p.record(task='t',map_digest='wrong',point_id='v0',outcome='observed_empty',now=1,cleanup_verified=True)
    def test_old_point_rejected(self):
        self.record()
        with self.assertRaises(ValueError):self.record(now=2)
    def test_exhausted_is_not_absence(self):
        self.record();self.record('v1',now=2);r=self.record('v2',now=3)
        self.assertEqual(r['state'],'exhausted');self.assertFalse(r['does_not_exist_authorized'])
    def test_cancel_does_not_resume(self):
        self.p.cancel();self.assertEqual(self.p.propose(1)['proposal'],'none')
    def test_clock_reversal(self):
        self.p.propose(2)
        with self.assertRaises(ValueError):self.p.propose(1)
    def test_fatal_never_skips(self):
        self.assertEqual(self.record(outcome='fatal')['state'],'blocked')
        self.assertEqual(self.p.propose(2)['point_id'],'v0')


if __name__=='__main__':unittest.main()
