import unittest
from synced_capture_selection import select_stamps


class Tests(unittest.TestCase):
    def test_wait_for_tf(self):
        h={k:[800_000_000,990_000_000] for k in ('rgb','depth','hand')}
        self.assertEqual(select_stamps(h,1_000_000_000),(800_000_000,)*3)
    def test_no_duplicate(self):
        h={k:[800_000_000] for k in ('rgb','depth','hand')}
        self.assertIsNone(select_stamps(h,1_000_000_000,(800_000_000,)*3))
    def test_stale(self):
        h={k:[400_000_000] for k in ('rgb','depth','hand')}
        self.assertIsNone(select_stamps(h,1_000_000_000))
    def test_hand_before_depth(self):
        self.assertIsNone(select_stamps(dict(rgb=[800_000_000],depth=[800_000_000],hand=[790_000_000]),1_000_000_000))
    def test_misaligned_rgb(self):
        self.assertIsNone(select_stamps(dict(rgb=[700_000_000],depth=[800_000_000],hand=[810_000_000]),1_000_000_000))
    def test_empty(self):self.assertIsNone(select_stamps(dict(rgb=[],depth=[],hand=[]),1_000_000_000))

    def test_ready_tf_uses_newest_without_fixed_100ms_delay(self):
        h={k:[800_000_000,980_000_000] for k in ('rgb','depth','hand')}
        self.assertEqual(select_stamps(h,1_000_000_000,ready=lambda stamps:True,
            joint_stamp_ns=990_000_000,processing_reserve_ns=200_000_000),(980_000_000,)*3)

    def test_unready_newest_uses_recent_pair_with_real_tf_not_guessed_time(self):
        h={k:[850_000_000,980_000_000] for k in ('rgb','depth','hand')}
        self.assertEqual(select_stamps(h,1_000_000_000,ready=lambda stamps:stamps[1]==850_000_000,
            joint_stamp_ns=990_000_000,processing_reserve_ns=200_000_000),(850_000_000,)*3)

    def test_no_ready_tf_does_not_authorize_fresh_pixels(self):
        h={k:[980_000_000] for k in ('rgb','depth','hand')}
        self.assertIsNone(select_stamps(h,1_000_000_000,ready=lambda stamps:False,joint_stamp_ns=990_000_000))

    def test_old_joint_with_fresh_pixels_waits_before_expensive_inference(self):
        h={k:[980_000_000] for k in ('rgb','depth','hand')}
        self.assertIsNone(select_stamps(h,1_000_000_000,ready=lambda stamps:True,
            joint_stamp_ns=700_000_000,processing_reserve_ns=200_000_000))

    def test_old_image_does_not_consume_processing_budget(self):
        h={k:[750_000_000] for k in ('rgb','depth','hand')}
        self.assertIsNone(select_stamps(h,1_000_000_000,ready=lambda stamps:True,
            joint_stamp_ns=990_000_000,processing_reserve_ns=200_000_000))

    def test_future_joint_is_not_forgiven_by_timestamp_rewrite(self):
        h={k:[980_000_000] for k in ('rgb','depth','hand')}
        self.assertIsNone(select_stamps(h,1_000_000_000,ready=lambda stamps:True,joint_stamp_ns=1_030_000_000))

    def test_new_mode_keeps_original_rgb_depth_sync_bound(self):
        h=dict(rgb=[940_000_000],depth=[980_000_000],hand=[990_000_000])
        self.assertIsNone(select_stamps(h,1_000_000_000,ready=lambda stamps:True,joint_stamp_ns=990_000_000))

    def test_new_mode_keeps_strictly_increasing_source_stamps(self):
        h={k:[980_000_000] for k in ('rgb','depth','hand')}
        self.assertIsNone(select_stamps(h,1_000_000_000,(980_000_000,)*3,ready=lambda stamps:True))


if __name__=='__main__':unittest.main()
