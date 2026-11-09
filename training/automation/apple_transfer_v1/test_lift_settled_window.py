import unittest
from lift_settled_window import bounds,contains


class WindowTests(unittest.TestCase):
    def sample(self,rgb=20,depth=20,hand=21):
        return dict(streams=dict(rgb=dict(stamp_ns=rgb),depth=dict(stamp_ns=depth),hand=dict(stamp_ns=hand)))
    def test_original_rgb_depth_hand_must_all_follow_measured_completion(self):
        self.assertTrue(contains(self.sample(),20,30))
        for frame in (self.sample(rgb=19),self.sample(depth=19),self.sample(hand=19)):
            self.assertFalse(contains(frame,20,30))
    def test_new_joint_cannot_retimestamp_old_camera(self):
        frame=self.sample(depth=19);frame['joint_stamp_ns']=25
        self.assertFalse(contains(frame,20,30));self.assertEqual(frame['streams']['depth']['stamp_ns'],19)
    def test_frozen_end_keeps_later_observations_out(self):
        for frame in (self.sample(rgb=31),self.sample(depth=31),self.sample(hand=31)):
            self.assertFalse(contains(frame,20,30))
    def test_missing_or_invalid_endpoint_proof_rejected(self):
        for trial in ({},{'lift_completed_at_ns':20},{'lift_completed_at_ns':False,'verification_window_end_ns':30},
                      {'lift_completed_at_ns':20,'verification_window_end_ns':20},
                      {'lift_completed_at_ns':20,'verification_window_end_ns':19}):
            with self.assertRaises(ValueError):bounds(trial)
    def test_valid_bounds_are_original_evidence_not_current_time(self):
        trial=dict(lift_completed_at_ns=20,verification_window_end_ns=30)
        self.assertEqual(bounds(trial),(20,30));self.assertEqual(trial['lift_completed_at_ns'],20)
    def test_invalid_camera_stamp_rejected(self):
        for stamp in (True,0,-1,float('nan'),20.):
            with self.assertRaises(ValueError):contains(self.sample(rgb=stamp),20,30)


if __name__=='__main__':unittest.main()
