import ast
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import head_view_trial as h


class HeadViewTests(unittest.TestCase):
    def setUp(self):self.g=h.Settling((0.,-.5))
    def sample(self,i,pose=(0.,-.5)):
        stamp=1_000_000_000+i*100_000_000
        return self.g.sample(list(h.JOINTS),pose,stamp,stamp,i*.1)
    def test_settles(self):
        for i in range(8):ok,pose=self.sample(i)
        self.assertTrue(ok)
    def test_not_immediate(self):self.assertFalse(self.sample(0)[0])
    def test_reason_tracks_stable_window(self):
        self.sample(0);self.assertEqual(self.g.reason,'collecting_stationary_window')
        for i in range(1,8):self.sample(i)
        self.assertEqual(self.g.reason,'settled')
    def test_reason_records_gap(self):
        self.sample(0);self.sample(5)
        self.assertEqual(self.g.reason,'feedback_gap')
    def test_reason_records_bad_timestamp(self):
        self.sample(0);self.sample(0)
        self.assertEqual(self.g.reason,'stale_or_invalid_feedback')
    def test_smaller_tilt(self):
        self.assertEqual(h.target(0.,-.25),(0.,-.25))
    def test_submillisecond_clock_skew_settles(self):
        for i in range(9):
            stamp=1_000_000_000+i*100_000_000
            ok,_=self.g.sample(list(h.JOINTS),(0.,-.5),stamp,stamp-732_000,i*.1)
        self.assertTrue(ok)
    def test_future_allowance_boundary(self):
        _,pose=self.g.sample(list(h.JOINTS),(0.,-.5),2_000_000_000,2_000_000_000-h.MAX_FUTURE_SKEW_NS,0.)
        self.assertIsNotNone(pose)
    def test_larger_future_skew_still_rejected(self):
        _,pose=self.g.sample(list(h.JOINTS),(0.,-.5),2_000_000_000,2_000_000_000-h.MAX_FUTURE_SKEW_NS-1,0.)
        self.assertIsNone(pose)
    def test_live_fourteen_ms_skew_settles(self):
        for i in range(9):
            stamp=1_000_000_000+i*100_000_000
            ok,_=self.g.sample(list(h.JOINTS),(0.,-.5),stamp,stamp-14_105_000,i*.1)
        self.assertTrue(ok)
    def test_skew_does_not_bypass_stale_limit(self):
        self.assertIsNone(self.g.sample(list(h.JOINTS),(0.,-.5),1_000_000_000,1_300_000_001,0.)[1])
    def test_allowance_does_not_accept_duplicates(self):
        args=(list(h.JOINTS),(0.,-.5),2_000_000_000,1_999_500_000)
        self.g.sample(*args,0.)
        self.assertIsNone(self.g.sample(*args,.1)[1])
    def test_allowance_does_not_accept_reversal(self):
        self.sample(5)
        stamp=1_400_000_000
        self.assertIsNone(self.g.sample(list(h.JOINTS),(0.,-.5),stamp,stamp-500_000,.6)[1])
    def test_outside_target(self):
        for i in range(10):ok,_=self.sample(i,(0.,0.))
        self.assertFalse(ok)
    def test_duplicate(self):
        self.sample(0);self.assertIsNone(self.sample(0)[1])
    def test_missing_joint(self):
        self.assertIsNone(self.g.sample(['head_pan_joint'],[0.],1,1,0.)[1])
    def test_duplicate_joint(self):
        self.assertIsNone(self.g.sample(['head_pan_joint']*2,[0.,0.],1,1,0.)[1])
    def test_stale(self):
        self.assertIsNone(self.g.sample(list(h.JOINTS),[0.,-.5],1,400_000_000,0.)[1])
    def test_nan(self):self.assertIsNone(self.sample(0,(0.,float('nan')))[1])
    def test_gap_restarts(self):
        for i in range(8):self.sample(i)
        self.assertFalse(self.sample(12)[0])
    def test_moving_restarts(self):
        for i in range(8):self.sample(i)
        self.assertFalse(self.sample(8,(0.,-.48))[0])
    def test_targets(self):
        self.assertEqual(h.target(0.,-.5),(0.,-.5))
        for p in [(1.,0.),(0.,.1),(0.,-1.),(float('nan'),0.)]:
            with self.assertRaises(ValueError):h.target(*p)
    def test_head_publisher_only(self):
        tree=ast.parse(Path(h.__file__).read_text())
        calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
               and n.func.attr=='create_publisher']
        self.assertEqual(len(calls),1)
        self.assertEqual(ast.unparse(calls[0].args[1]),'COMMAND')
        self.assertEqual(h.COMMAND,'/hsrb/head_trajectory_controller/command')


if __name__=='__main__':unittest.main()
