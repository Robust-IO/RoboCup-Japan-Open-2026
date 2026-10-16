import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_timing import session_budget,remaining,stops_session,SessionEvents
from search_scan import SearchScan


class TimingTests(unittest.TestCase):
    def test_valid_session_budget(self):self.assertEqual(session_budget(180),180.)
    def test_invalid_budget(self):
        for v in (0,-1,601,True,float('inf'),float('nan')):
            with self.subTest(v=v),self.assertRaises(ValueError):session_budget(v)
    def test_remaining_is_not_renewed(self):
        self.assertEqual(remaining(100,40),60)
        self.assertEqual(remaining(100,60),40)
        with self.assertRaises(ValueError):remaining(100,100)
    def test_observation_starts_after_long_navigation(self):
        scan=SearchScan('task','apple',[dict(x=0,y=0,yaw=0)],0,timeout_s=180,view_s=5)
        scan.tick(65);self.assertEqual(scan.phase,'navigate')
        scan.arrived('task:0',True,True,65)
        scan.tick(69.9);self.assertEqual(scan.phase,'observe')
        scan.tick(70);self.assertEqual(scan.phase,'incomplete')
        self.assertEqual(scan.failures,['0:observation_timeout'])
    def test_session_expiry_still_stops_navigation(self):
        scan=SearchScan('task','apple',[dict(x=0,y=0,yaw=0)],0,timeout_s=60,view_s=5)
        scan.tick(60);self.assertEqual(scan.failures,['search_timeout'])
    def test_session_stop_events(self):
        for event in ('Task_failed','Task_succeeded','Mission_complete'):
            self.assertTrue(stops_session(event))
        for event in ('Are_you_ready?','Instruction','Room_reached','unrelated'):self.assertFalse(stops_session(event))
    def test_environment_announcements(self):
        state=SessionEvents()
        self.assertFalse(state.stops('Environment','LayoutA'))
        self.assertFalse(state.stops('Are_you_ready?'))
        self.assertFalse(state.stops('Environment','LayoutA'))
        self.assertTrue(state.stops('Environment','LayoutB'))
        self.assertTrue(state.stops('Mission_complete'))
    def test_empty_environment_fails_closed(self):
        self.assertTrue(SessionEvents().stops('Environment',''))
