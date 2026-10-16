import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from search_process_watchdog import SearchProcessWatchdog


class Process:
    code = None
    def poll(self): return self.code


class WatchdogTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.process = Process()
        self.watch = SearchProcessWatchdog('a'*32, self.process, 2, clock=lambda: self.now)
        self.row = dict(schema='handyman-search-request-v1', task_id='a'*32, cancel_id='b'*32)

    def test_running_never_certifies_drain(self):
        self.assertIsNone(self.watch.fault_notice())
        self.assertTrue(self.watch.cancellation(self.row))
        self.assertIsNone(self.watch.failure_report())

    def test_exit_codes_all_fail(self):
        for code in (0, 7, -15):
            with self.subTest(code=code):
                self.setUp(); self.process.code = code
                self.assertIsNone(self.watch.failure_report())
                self.assertTrue(self.watch.cancellation(self.row))
                self.assertEqual(self.watch.failure_report()['state'], 'cancel_failed')
                self.assertEqual(self.watch.failure_report()['returncode'], code)

    def test_deadline_does_not_signal_process(self):
        self.now = 12.
        self.watch.cancellation(self.row)
        self.assertEqual(self.watch.failure_report()['reason'], 'worker_lifetime_exceeded')
        self.assertIsNone(self.process.poll())

    def test_fault_sticky(self):
        self.process.code = 7; self.watch.sample(); self.process.code = None
        self.watch.cancellation(self.row)
        self.assertEqual(self.watch.failure_report()['returncode'], 7)

    def test_fault_notice_precedes_cancel_context(self):
        self.process.code = 7
        notice = self.watch.fault_notice()
        self.assertEqual(notice['schema'], 'handyman-search-worker-fault-v1')
        self.assertEqual(notice['task_id'], 'a'*32)
        self.assertNotIn('cancel_id', notice)
        self.assertIsNone(self.watch.failure_report())

    def test_wrong_cancel_ignored(self):
        self.process.code = 7
        for row in (None, {}, dict(self.row, task_id='c'*32), dict(self.row, cancel_id='0'*32)):
            self.assertFalse(self.watch.cancellation(row))
        self.assertIsNone(self.watch.failure_report())

    def test_duplicate_does_not_extend_deadline(self):
        self.watch.cancellation(self.row); self.now = 12.
        self.assertTrue(self.watch.cancellation(self.row))
        self.assertFalse(self.watch.cancellation(dict(self.row, cancel_id='c'*32)))
        self.assertEqual(self.watch.failure_report()['cancel_id'], 'b'*32)

    def test_invalid_configuration(self):
        for lifetime in (0, -1, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                SearchProcessWatchdog('a'*32, self.process, lifetime)
        with self.assertRaises(ValueError):
            SearchProcessWatchdog('0'*32, self.process, 2)

    def drain(self):
        return dict(schema='handyman-search-cancel-status-v1',task_id='a'*32,cancel_id='b'*32,state='cancel_drained')

    def test_confirmed_retirement_allows_zero_exit(self):
        self.watch.cancellation(self.row)
        self.assertTrue(self.watch.prepare_retirement(self.drain()))
        self.process.code=0
        self.assertIsNone(self.watch.sample());self.assertTrue(self.watch.retired)
        self.now=100.;self.assertIsNone(self.watch.sample())

    def test_exit_before_retirement_remains_fault(self):
        self.watch.cancellation(self.row);self.process.code=0
        self.assertFalse(self.watch.prepare_retirement(self.drain()))
        self.assertIsNotNone(self.watch.sample())

    def test_wrong_or_failed_drain_not_authorized(self):
        self.assertFalse(self.watch.prepare_retirement(self.drain()))
        self.watch.cancellation(self.row)
        for row in (dict(self.drain(),task_id='c'*32),dict(self.drain(),cancel_id='c'*32),
                    dict(self.drain(),state='cancel_failed')):
            self.assertFalse(self.watch.prepare_retirement(row))

    def test_duplicate_retirement_does_not_extend(self):
        self.watch.cancellation(self.row);self.watch.prepare_retirement(self.drain())
        limit=self.watch.retirement_deadline;self.now+=.5
        self.assertTrue(self.watch.prepare_retirement(self.drain()))
        self.assertEqual(limit,self.watch.retirement_deadline)
        self.now=limit;self.assertIsNotNone(self.watch.sample())

    def test_abnormal_exit_after_permission_still_faults(self):
        self.watch.cancellation(self.row);self.watch.prepare_retirement(self.drain())
        self.process.code=-9;self.assertIsNotNone(self.watch.sample())


if __name__ == '__main__':
    unittest.main()
