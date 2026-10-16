import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from observe_current_view import CurrentView
from rgbd_localization import AUDIT_SHA256


def row(i,present=True):
    t=1_000_000_000+i*100_000_000
    return dict(target='canned_juice',geometry_verified=True,audit_sha256=AUDIT_SHA256,
        rgb_stamp_ns=t,depth_stamp_ns=t,tf_wait_status='ready',tf_at_depth_stamp={},
        quality=dict(stable=present,position_m=[1,2,3],frame_id='odom',target='canned_juice',stamp_ns=t),
        inference=dict(detections=[dict(name='canned_juice')] if present else [],class_conflicts=[],
            view_health=dict(schema='handyman-view-health-v1',data_usable=True,coverage_verified=False)))


class Tests(unittest.TestCase):
    def test_found_only_after_new_frames(self):
        o=CurrentView('canned_juice',0,1_000_000_000,10)
        for i in range(1,4):r=o.consume(row(i),i*.1,1_000_000_000+i*100_000_000)
        self.assertEqual(r['state'],'found');self.assertEqual(r['position_m'],[1,2,3])
        self.assertFalse(r['navigation_arrival_verified']);self.assertFalse(r['actionable'])
    def test_absence_not_room_absence(self):
        o=CurrentView('canned_juice',0,1_000_000_000,10)
        for i in range(1,4):r=o.consume(row(i,False),i*.1,1_000_000_000+i*100_000_000)
        self.assertEqual(r['state'],'observe');self.assertFalse(r['does_not_exist_authorized'])
        self.assertEqual(o.scan.tick(11)['state'],'incomplete')
    def test_old_frames_rejected(self):
        o=CurrentView('canned_juice',0,2_000_000_000,10)
        for i in range(1,4):r=o.consume(row(i),i*.1,2_100_000_000)
        self.assertEqual(r['state'],'observe');self.assertIsNone(r['position_m'])
    def test_malformed_breaks_streak(self):
        o=CurrentView('canned_juice',0,1_000_000_000,10)
        o.consume(row(1),.1,1_100_000_000);o.consume({},.15,1_150_000_000)
        r=o.consume(row(2),.2,1_200_000_000)
        self.assertEqual(r['fresh_view_count'],1)


if __name__=='__main__':unittest.main()
