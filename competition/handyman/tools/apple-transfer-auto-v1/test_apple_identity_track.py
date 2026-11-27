import unittest
from apple_identity_track import AppleIdentityTrack


class Tests(unittest.TestCase):
    def setUp(self):
        self.track=AppleIdentityTrack('task','epoch')
        self.stamp=1000000000

    def frame(self, confidence=.95, **changes):
        self.stamp+=100000000
        item=dict(point_odom_m=[1.,2.,.48],radius_m=.03,confidence=confidence,
                  fit_residual_p95_m=.005,measurement='fresh_rgbd',stamp_ns=self.stamp,
                  class_name='apple',red_appearance=True,clipped=False,depth_valid=True,
                  support_consistent=True)
        item.update(changes)
        return item

    def run_frame(self,item,**kw):
        args=dict(stamp_ns=self.stamp,now_ns=self.stamp+10000000,
                  task_id='task',epoch='epoch',room_confirmed=True)
        args.update(kw)
        return self.track.update([item],**args)

    def seed(self):
        for _ in range(3):result=self.run_frame(self.frame())
        self.assertTrue(result['identity_confirmed'])

    def test_low_confidence_continuation(self):
        self.seed()
        r=self.run_frame(self.frame(.31))
        self.assertEqual(r['status'],'identity_tracked')
        self.assertFalse(r['motion_authorized'])

    def test_low_confidence_cannot_seed(self):
        self.assertEqual(self.run_frame(self.frame(.31))['status'],'reobserve')

    def test_gap_requires_new_identity(self):
        self.seed();self.stamp+=1000000000
        self.assertEqual(self.run_frame(self.frame(.31))['reason'],'identity_reconfirmation_required_after_gap')

    def test_gap_with_new_strong_same_geometry_reconfirms_without_throwing_away_frame(self):
        self.seed();self.stamp+=1000000000
        result=self.run_frame(self.frame(.95))
        self.assertTrue(result['identity_confirmed'])
        self.assertTrue(result['gap_reconfirmed_with_current_classification'])

    def test_gap_does_not_allow_new_location_to_claim_old_identity(self):
        self.seed();self.stamp+=1000000000
        self.assertFalse(self.run_frame(self.frame(.95,point_odom_m=[2,2,.48]))['identity_confirmed'])

    def test_gap_reconfirmation_cannot_accept_stale_or_predicted_measurement(self):
        for mutation in ('stale','predicted'):
            self.setUp();self.seed();self.stamp+=1000000000
            item=self.frame(.95)
            if mutation=='predicted':item['measurement']='predicted'
            kwargs=dict(now_ns=self.stamp+500000000) if mutation=='stale' else {}
            with self.subTest(mutation=mutation):self.assertFalse(self.run_frame(item,**kwargs)['identity_confirmed'])

    def test_moved_target_rejected(self):
        self.seed()
        self.assertEqual(self.run_frame(self.frame(.31,point_odom_m=[1.1,2.,.48]))['status'],'reobserve')

    def test_predicted_point_cannot_refresh(self):
        self.seed()
        self.assertEqual(self.run_frame(self.frame(.31,measurement='predicted'))['status'],'reobserve')

    def test_stale(self):
        self.seed();item=self.frame()
        self.assertEqual(self.run_frame(item,now_ns=self.stamp+500000000)['reason'],'stale_observation')

    def test_epoch_change(self):
        self.seed()
        self.assertEqual(self.run_frame(self.frame(),epoch='new')['status'],'reobserve')

    def test_ambiguous_low_score_twin(self):
        self.seed();item=self.frame(.31)
        r=self.track.update([item,dict(item,confidence=.1)],stamp_ns=self.stamp,
            now_ns=self.stamp+10000000,task_id='task',epoch='epoch',room_confirmed=True)
        self.assertEqual(r['reason'],'missing_or_ambiguous_association')

    def test_contact_invalidates(self):
        self.seed()
        self.assertEqual(self.run_frame(self.frame(),contact_started=True)['status'],'reobserve')

    def test_depth_required(self):
        self.seed()
        self.assertEqual(self.run_frame(self.frame(depth_valid=False))['status'],'reobserve')


if __name__=='__main__':unittest.main()
