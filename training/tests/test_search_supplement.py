"""Regression for the fourth live direction, without ROS or Unity."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_view_evidence import ViewEvidence
from search_scan import SearchScan


class SupplementTests(unittest.TestCase):
    def setUp(self):
        self.e=ViewEvidence();self.e.arrived(100)
        self.s=SearchScan('t','can',[dict(x=0,y=0,yaw=0)],0,timeout_s=20,view_s=3)
        self.s.evidence_ready=lambda now:self.e.snapshot(now)['view_data_usable']
        self.s.arrived('t:0',True,True,0)

    def frame(self,stamp,now):
        self.s.tick(now)
        self.e.observe(dict(depth_stamp_ns=stamp),dict(self.s.status(),
            adapter_reason='no_target_data_usable_but_coverage_unverified'),now)

    def test_live_four_frames_survive_gaps_and_final_age(self):
        for stamp,now in zip([101,102,103,104],[0,.573813727,1.173193523,1.325905144]):
            self.frame(stamp,now)
        self.s.tick(3.027530357)
        result=self.e.snapshot(3.027530357)
        self.assertEqual(len(result['negative_stamps']),4)
        self.assertTrue(result['view_data_usable'])
        self.assertFalse(result['current_data_fresh'])
        self.assertEqual(self.s.failures,['0:observation_timeout'])

    def test_supplement_until_third_frame(self):
        self.frame(101,.1);self.frame(102,1)
        self.s.tick(3.1);self.assertEqual(self.s.phase,'observe')
        self.frame(103,4)
        self.s.tick(4.01);self.assertEqual(self.s.phase,'incomplete')
        self.assertTrue(self.e.snapshot(4.01)['view_data_usable'])

    def test_supplement_never_exceeds_three_extra_seconds(self):
        self.s.tick(5.99);self.assertEqual(self.s.phase,'observe')
        self.s.tick(6);self.assertEqual(self.s.phase,'incomplete')
        self.assertFalse(self.e.snapshot(6)['view_data_usable'])

    def test_overall_deadline_wins(self):
        self.s.deadline=4;self.s.tick(4)
        self.assertEqual(self.s.failures,['search_timeout'])

    def test_invalid_data_clears_completed_evidence(self):
        for i in (1,2,3):self.frame(100+i,i*.1)
        self.e.observe({},dict(state='observe',adapter_reason='missing_tf'),.4)
        self.assertFalse(self.e.snapshot(.4)['view_data_usable'])

    def test_new_arrival_clears_history(self):
        for i in (1,2,3):self.frame(100+i,i*.1)
        self.e.arrived(200)
        self.assertEqual(self.e.snapshot(1)['negative_stamps'],[])

    def test_late_frame_not_counted(self):
        self.frame(101,.1);self.frame(102,1);self.frame(103,6.1)
        self.assertFalse(self.e.snapshot(6.1)['view_data_usable'])

    def test_cancel_not_extended(self):
        self.s.cancel();self.s.tick(4)
        self.assertEqual(self.s.phase,'cancelled')
