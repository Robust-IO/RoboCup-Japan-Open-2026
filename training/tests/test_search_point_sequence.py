import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_point_sequence import SearchPointSequence


class Tests(unittest.TestCase):
    def setUp(self):self.s=SearchPointSequence('t','canned_juice','digest',['p0','p1'])
    def row(self,point='p0',**changes):
        return dict(dict(task_id='t',target='canned_juice',observer_context=dict(task_id='t',point_id=point,map_sha256='digest'),
            state='incomplete',failures=['0:observation_timeout'],cleanup_verified=True,terminal=True,
            actionable=False,does_not_exist_authorized=False),**changes)
    def feed(self,row=None,**changes):
        args=dict(arrival_verified=True,negative_stamps=[1,2,3],view_data_usable=True);args.update(changes)
        return self.s.complete(self.row() if row is None else row,**args)
    def test_next_point(self):
        r=self.feed();self.assertEqual(r['point_id'],'p1');self.assertEqual(r['state'],'awaiting_point')
    def test_cleanup_pending(self):
        r=self.feed(self.row(cleanup_verified=False));self.assertEqual(r['point_id'],'p0')
    def test_old_point_rejected(self):
        self.feed()
        with self.assertRaises(ValueError):self.feed()
    def test_found_after_next(self):
        self.feed();r=self.feed(self.row('p1',state='found',failures=[],position_m=[1.,2.,3.],position_frame='odom'))
        self.assertEqual(r['state'],'found');self.assertEqual(r['position_m'],[1.,2.,3.])
    def test_exhaustion_not_absence(self):
        self.feed();r=self.feed(self.row('p1'))
        self.assertEqual(r['state'],'incomplete');self.assertFalse(r['does_not_exist_authorized'])
    def test_navigation_timeout_not_negative(self):
        r=self.feed(self.row(failures=['search_timeout']))
        self.assertEqual(r['state'],'incomplete');self.assertEqual(r['point_id'],'p0')
    def test_bad_view_blocks(self):self.assertEqual(self.feed(view_data_usable=False)['state'],'incomplete')
    def test_no_arrival_blocks(self):self.assertEqual(self.feed(arrival_verified=False)['state'],'incomplete')
    def test_duplicate_negatives_block(self):self.assertEqual(self.feed(negative_stamps=[1,1,2])['state'],'incomplete')
    def test_cancellation_latches(self):
        self.s.cancel();self.assertEqual(self.feed()['state'],'cancelled')
    def test_fault_latches(self):
        self.feed(view_data_usable=False);self.assertEqual(self.feed()['state'],'incomplete')
    def test_wrong_map(self):
        r=self.row();r['observer_context']['map_sha256']='other'
        with self.assertRaises(ValueError):self.feed(r)


if __name__=='__main__':unittest.main()
