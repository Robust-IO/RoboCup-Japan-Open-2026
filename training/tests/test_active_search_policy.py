import math
import unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from active_search_policy import ActiveSearch, ring_views


class ActiveSearchTests(unittest.TestCase):
    def make(self):
        return ActiveSearch('task', 'map', 'room', 'can', ['v0', 'v1'], ['table'], 100)

    def feed(self, s, score=.9, x=1., instance='a', start=1):
        for t in range(start, start+3):
            s.observe(s.identity, 'v0', t, 1, True,
                      [dict(label='can', score=score, instance_id=instance, position=[x, 0, 1])])

    def test_ring(self):
        ring=ring_views(1,2,0)
        self.assertEqual(len(ring),8)
        self.assertAlmostEqual(ring[0]['overlap_deg'],14.99)
        self.assertTrue(all(-math.pi<=v['yaw']<=math.pi for v in ring))

    def test_invalid_ring(self):
        for args in [(0,0,float('nan')), (0,0,0,60,60)]:
            with self.assertRaises(ValueError): ring_views(*args)

    def test_high_priority_and_resume(self):
        s=self.make();self.feed(s)
        self.assertEqual(s.next_action(2)['action'],'verify')
        s.verify('object/0','front','rejected')
        self.assertEqual(s.next_action(3)['view'],'v0')

    def test_false_top_does_not_discard_lower(self):
        s=self.make();self.feed(s);self.feed(s,.4,2.,'b',4)
        s.finish_view('v0',True);s.finish_view('v1',True)
        s.verify('object/0','front','rejected')
        self.assertEqual(s.next_action(3)['candidate'],'object/1')

    def test_frames_not_double_counted(self):
        s=self.make();self.feed(s);self.feed(s)
        self.assertEqual(len(s.candidates['object/0'].scores),3)

    def test_two_close_instances_same_frame(self):
        s=self.make()
        s.observe(s.identity,'v0',1,1,True,[dict(label='can',score=.9,instance_id=i,position=[x,0,1])
                                          for i,x in [('a',1),('b',1.05)]])
        self.assertEqual(len(s.candidates),2)

    def test_wrong_identity_and_class(self):
        s=self.make()
        self.assertEqual(s.observe(('other','map','room'),'v0',1,1,True,[]),[])
        s.observe(s.identity,'v0',2,1,True,[dict(label='cup',score=.99,instance_id='a')])
        self.assertFalse(s.candidates)

    def test_no_absence_after_ring(self):
        s=self.make();s.finish_view('v0',True);s.finish_view('v1',True)
        self.assertEqual(s.next_action(2)['action'],'supplement')
        s.mark_region('table','reviewed')
        self.assertEqual(s.next_action(2)['action'],'review_completion')
        self.assertFalse(s.next_action(2)['does_not_exist_authorized'])

    def test_navigation_failure_not_rejection(self):
        s=self.make();self.feed(s)
        s.verify('object/0','front','navigation_failed')
        self.assertEqual(s.candidates['object/0'].state,'deferred')
        self.assertFalse(s.verify('object/0','front','rejected'))

    def test_expiry_and_cancel(self):
        s=self.make();self.assertEqual(s.next_action(100)['action'],'incomplete')
        s=self.make();s.cancel();self.feed(s)
        self.assertFalse(s.candidates)
        self.assertEqual(s.next_action(1)['action'],'cancelled')

    def test_missing_depth_is_not_navigation(self):
        s=self.make()
        for t in range(1,4):
            s.observe(s.identity,'v0',t,1,True,[dict(label='can',score=.9,instance_id='a')])
        self.assertEqual(s.next_action(2)['action'],'resolve_depth')
        with self.assertRaises(ValueError):s.verify('bearing/v0/a','front','confirmed')

    def test_invalid_view_prevents_completion(self):
        s=self.make();s.finish_view('v0',True);s.finish_view('v1',False)
        s.mark_region('table','reviewed')
        self.assertEqual(s.next_action(2)['action'],'supplement')
        self.assertEqual(s.next_action(2)['invalid_views'],['v1'])

    def test_unhealthy_frame_ignored(self):
        s=self.make()
        s.observe(s.identity,'v0',1,1,False,[dict(label='can',score=.9,instance_id='a')])
        self.assertFalse(s.candidates)

    def test_rejected_candidate_not_reactivated_by_same_geometry(self):
        s=self.make();self.feed(s);s.verify('object/0','front','rejected');self.feed(s,start=4)
        self.assertEqual(len(s.candidates),1)
        self.assertEqual(s.candidates['object/0'].state,'rejected')

    def test_cross_view_geometry_merge(self):
        s=self.make();self.feed(s)
        s.observe(s.identity,'v1',4,2,True,[dict(label='can',score=.7,instance_id='other',position=[1.01,0,1])])
        self.assertEqual(len(s.candidates),1)
        self.assertEqual(s.candidates['object/0'].views,{'v0','v1'})


if __name__=='__main__': unittest.main()
