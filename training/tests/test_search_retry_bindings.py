from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_arrival_ros import RetryBindings,GoalStatusGate


class RetryTests(unittest.TestCase):
    def setUp(self):
        self.r=RetryBindings(2)
        self.first=dict(goal_id='1'*32,executor_generation=2)
        self.second=dict(goal_id='2'*32,executor_generation=4)
        self.r.accept(self.first)

    def test_abort_allows_new(self):
        self.assertTrue(self.r.failed(6));self.r.accept(self.second)
        self.assertFalse(self.r.failed(6))

    def test_cancel_terminal(self):self.assertFalse(self.r.failed(5))

    def test_no_unsolicited_rebind(self):
        with self.assertRaises(ValueError):self.r.accept(self.second)

    def test_replay_rejected(self):
        self.r.failed(6)
        for value in (self.first,dict(self.second,executor_generation=1),dict(self.second,goal_id='1'*32)):
            with self.assertRaises(ValueError):self.r.accept(value)
        self.r.accept(self.second)

    def test_old_result_ignored(self):
        self.r.failed(6);self.r.accept(self.second)
        status=GoalStatusGate(self.second['goal_id']);status.active=True
        self.assertIsNone(status.consume(self.first['goal_id'],4))
        self.assertTrue(status.consume(self.second['goal_id'],4))

    def test_defer_then_abort(self):
        self.r.defer(self.second,'message',1.)
        self.assertEqual(self.r.seen,{'1'*32})
        self.r.failed(6)
        self.assertEqual(self.r.take_pending(1.5),'message')
        self.r.accept(self.second)

    def test_pending_expiry(self):
        self.r.defer(self.second,'message',1.)
        self.r.defer(self.second,'message',2.)
        self.r.failed(6)
        self.assertIsNone(self.r.take_pending(3.01))

    def test_cancel_discards_pending(self):
        self.r.defer(self.second,'message',1.)
        self.r.failed(5)
        self.assertIsNone(self.r.take_pending(1.5))

    def test_pending_conflict(self):
        self.r.defer(self.second,'message',1.)
        with self.assertRaises(ValueError):
            self.r.defer(dict(goal_id='3'*32,executor_generation=5),'other',1.1)

    def test_pending_does_not_authorize(self):
        self.r.defer(self.second,'message',1.)
        self.assertIsNone(self.r.take_pending(1.1))
        with self.assertRaises(ValueError):self.r.accept(self.second)

if __name__=='__main__':unittest.main()
