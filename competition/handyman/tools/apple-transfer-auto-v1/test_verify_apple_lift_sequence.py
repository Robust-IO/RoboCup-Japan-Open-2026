import copy
import unittest
from verify_apple_lift_sequence import pair


def frame(stamp,lift,z,opening):
    observation=dict(identity_tracking={'identity_confirmed':True},detections=[dict(name='apple',confidence=.46)])
    snapshot=dict(tracking_epoch='same',joints=dict(arm_lift_joint=lift,hand_motor_joint=opening,
        arm_flex_joint=-1.7,arm_roll_joint=0,wrist_flex_joint=.1292,wrist_roll_joint=0),
        base_from_hand=dict(translation=[.627,.078,.53+lift-.25],quaternion=[0,0,0,1]),
        palm_from_hand=dict(translation=[0,0,0],quaternion=[0,0,0,1]),
        tracking_transforms={'odom_from_base_at_depth':dict(stamp_ns=stamp,source_frame='base_link',target_frame='odom',translation=[0,0,0],quaternion=[0,0,0,1])})
    candidate=dict(stamp_ns=stamp,center_m=[.705,.078,z],radius_m=.03)
    return observation,snapshot,candidate


class Tests(unittest.TestCase):
    def setUp(self):
        self.before=frame(1_000_000_000,.25,.54,.65)
        self.after=frame(9_000_000_000,.35,.64,.415)
    def test_measured_apple_follows_palm_despite_low_score(self):
        r=pair(self.before,self.after,.415)
        self.assertAlmostEqual(r['object_delta_odom_m'][2],.1)
        self.assertLess(r['object_palm_follow_error_m'],1e-10)
    def test_apple_left_on_table(self):
        self.after[2]['center_m'][2]=.54
        with self.assertRaises(ValueError):pair(self.before,self.after,.415)
    def test_gripper_opened(self):
        self.after[1]['joints']['hand_motor_joint']=.65
        with self.assertRaises(ValueError):pair(self.before,self.after,.415)
    def test_epoch_restarted(self):
        self.after[1]['tracking_epoch']='new'
        with self.assertRaises(ValueError):pair(self.before,self.after,.415)
    def test_base_motion_is_not_failure_if_object_follows_measured_palm(self):
        self.after[1]['tracking_transforms']['odom_from_base_at_depth']['translation'][0]=.05
        self.assertLess(pair(self.before,self.after,.415)['object_palm_follow_error_m'],1e-10)


if __name__=='__main__':unittest.main()
