import copy
import unittest
from pepper_source_observation import source_identity_evidence
from canned_carry_geometry import current_images_and_feedback


class SourceIdentityTests(unittest.TestCase):
    def setUp(self):
        self.now=10_000_000_000
        names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        joints=[0.,0.,0.,-1.5707963267948966,0.,.65]
        self.snapshot=dict(tracking_epoch='current',streams={k:dict(stamp_ns=self.now-700_000_000) for k in ('rgb','depth')},
            joint_stamp_ns=self.now-900_000_000,joints=dict(zip(names,joints)),tracking_transforms=dict(
                odom_from_base_at_depth=dict(stamp_ns=self.now-700_000_000,translation=[1.,2.,0.],quaternion=[0.,0.,0.,1.])))
        self.snapshot['joints'].update(head_pan_joint=0.,head_tilt_joint=-.5)
        self.state=dict(stamp_ns=self.now-500_000_000,joints=joints,poses=dict(odom=[1.,2.,0.]))
    def evidence(self):return source_identity_evidence(self.snapshot,self.state,[0.,-.5],self.now,'current')
    def test_processed_identity_does_not_relax_action_depth(self):
        before=copy.deepcopy(self.snapshot);proof=self.evidence()
        self.assertTrue(proof['new_action_time_depth_still_required']);self.assertFalse(proof['motion_authorized'])
        self.assertAlmostEqual(proof['oldest_image_age_s'],.7);self.assertEqual(before,self.snapshot)
        with self.assertRaisesRegex(ValueError,'fresh_carry_images_required'):
            current_images_and_feedback(self.snapshot,self.state,self.now,'current',joint_max_age_ns=600_000_000)
    def test_expired_capture_and_epoch_rejected(self):
        with self.assertRaisesRegex(ValueError,'epoch'):
            source_identity_evidence(self.snapshot,self.state,[0.,-.5],self.now,'different')
        for stream in self.snapshot['streams'].values():stream['stamp_ns']=self.now-2_000_000_001
        with self.assertRaisesRegex(ValueError,'expired'):self.evidence()
    def test_stale_current_feedback_rejected(self):
        self.state['stamp_ns']=self.now-600_000_001
        with self.assertRaisesRegex(ValueError,'current_feedback'):self.evidence()
    def test_base_arm_and_head_motion_rejected(self):
        for field,change in (('base',.021),('yaw',.031),('arm',.011),('hand',-.04)):
            with self.subTest(field=field):
                original=copy.deepcopy(self.state)
                if field=='base':self.state['poses']['odom'][0]+=change
                elif field=='yaw':self.state['poses']['odom'][2]+=change
                else:self.state['joints'][0 if field=='arm' else 5]+=change
                with self.assertRaises(ValueError):self.evidence()
                self.state=original
        with self.assertRaisesRegex(ValueError,'head_moved'):
            source_identity_evidence(self.snapshot,self.state,[0.,-.469],self.now,'current')
    def test_image_time_transform_must_match_original_depth(self):
        self.snapshot['tracking_transforms']['odom_from_base_at_depth']['stamp_ns']+=1
        with self.assertRaisesRegex(ValueError,'image_time_pose'):self.evidence()

if __name__=='__main__':unittest.main()
