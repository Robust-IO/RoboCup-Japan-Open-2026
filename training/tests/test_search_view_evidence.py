import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_view_evidence import ViewEvidence
from search_point_sequence import SearchPointSequence


class Tests(unittest.TestCase):
    def setUp(self):self.e=ViewEvidence();self.e.arrived(100)
    def feed(self,i,reason='no_target_data_usable_but_coverage_unverified'):
        self.e.observe(dict(depth_stamp_ns=100+i),dict(state='observe',adapter_reason=reason),i*.1)
    def good(self):
        for i in (1,2,3):self.feed(i)
    def test_valid(self):
        self.good();self.assertTrue(self.e.snapshot(.35)['view_data_usable'])
    def test_stale(self):
        self.good();self.assertTrue(self.e.snapshot(1.)['view_data_usable'])
        self.assertFalse(self.e.snapshot(1.)['current_data_fresh'])
    def test_invalid_breaks_streak(self):
        self.good();self.feed(4,'missing_tf');self.assertEqual(self.e.snapshot(.4)['negative_stamps'],[])
    def test_arrival_required(self):
        self.e.reset();self.good();self.assertFalse(self.e.snapshot(.35)['view_data_usable'])
    def test_reset(self):
        self.good();self.e.arrived(200);self.assertEqual(self.e.snapshot(.35)['negative_stamps'],[])
    def test_duplicate(self):
        self.good();self.feed(3);self.assertFalse(self.e.snapshot(.35)['view_data_usable'])
    def test_deadline_frame_preserves_but_does_not_extend_evidence(self):
        self.good()
        self.e.observe(dict(depth_stamp_ns=999),dict(state='incomplete',failures=['0:observation_timeout']),.4)
        self.assertEqual(self.e.snapshot(.4)['negative_stamps'],[101,102,103])
        self.assertTrue(self.e.snapshot(1.)['view_data_usable'])
        self.assertFalse(self.e.snapshot(1.)['current_data_fresh'])
    def test_other_terminal_failure_does_not_preserve_evidence(self):
        self.good()
        self.e.observe({},dict(state='incomplete',failures=['search_timeout']),.4)
        self.assertFalse(self.e.snapshot(.4)['view_data_usable'])
    def test_next_point_from_terminal_evidence(self):
        self.good();s=SearchPointSequence('t','juice','map',['a','b'])
        r=dict(task_id='t',target='juice',observer_context=dict(task_id='t',point_id='a',map_sha256='map'),
               state='incomplete',failures=['0:observation_timeout'],terminal=True,cleanup_verified=True,
               actionable=False,does_not_exist_authorized=False,view_evidence=self.e.snapshot(.35))
        self.assertEqual(s.complete_observation(r)['point_id'],'b')


if __name__=='__main__':unittest.main()
