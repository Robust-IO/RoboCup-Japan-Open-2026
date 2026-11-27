import unittest
from unittest.mock import patch
from apple_tracking_observer import TrackingObserver


class Tests(unittest.TestCase):
    def setUp(self):
        self.observer=TrackingObserver('task','epoch',{},'living_room')
        self.stamp=1000000000
        self.room_patch=patch('apple_tracking_observer.room_check',return_value={'room_confirmed':True})
        self.room_patch.start();self.addCleanup(self.room_patch.stop)

    def inputs(self, confidence=.95):
        self.stamp+=100000000
        candidate=dict(stamp_ns=self.stamp,center_m=[1.,2.,.48],radius_m=.03,fit_residual_p95_m=.005)
        proposal=dict(candidate=candidate,detection=dict(name='apple',confidence=confidence,box=[100,100,150,150]),support={'support_consistent':True})
        result=dict(timing={'freshness_passed':True},tracking_proposals=[proposal])
        snapshot=dict(tracking_epoch='epoch',streams={'depth':{'stamp_ns':self.stamp},'rgb':{'stamp_ns':self.stamp,'width':640,'height':480}},
            tracking_transforms={'odom_from_base_at_depth':dict(stamp_ns=self.stamp,source_frame='base_link',target_frame='odom',translation=[0,0,0],quaternion=[0,0,0,1])})
        return result,snapshot

    def observe(self, confidence=.95):
        r,s=self.inputs(confidence)
        return self.observer.observe(r,s,self.stamp+10000000)

    def test_three_seed_then_low_score(self):
        for _ in range(3):self.observe()
        r=self.observe(.31)
        self.assertTrue(r['identity_confirmed']);self.assertFalse(r['motion_authorized'])

    def test_low_score_alone_not_identity(self):
        self.assertFalse(self.observe(.31)['identity_confirmed'])

    def test_bad_proposal_resets(self):
        for _ in range(3):self.observe()
        r,s=self.inputs();r['tracking_proposals'].append({'rejected_reason':'no_depth'})
        self.assertEqual(self.observer.observe(r,s,self.stamp)['reason'],'unresolved_geometry_proposal')
        self.assertFalse(self.observe(.31)['identity_confirmed'])

    def test_tf_wrong_time(self):
        r,s=self.inputs();s['tracking_transforms']['odom_from_base_at_depth']['stamp_ns']-=1
        self.assertEqual(self.observer.observe(r,s,self.stamp)['status'],'reobserve')

    def test_room_unverified(self):
        r,s=self.inputs()
        with patch('apple_tracking_observer.room_check',return_value={'room_confirmed':False}):
            self.assertEqual(self.observer.observe(r,s,self.stamp)['reason'],'proposal_room_unverified')

    def test_old_rgb_rejected(self):
        r,s=self.inputs();s['streams']['rgb']['stamp_ns']-=500000000
        self.assertEqual(self.observer.observe(r,s,self.stamp)['reason'],'rgbd_result_expired')


if __name__=='__main__':unittest.main()
