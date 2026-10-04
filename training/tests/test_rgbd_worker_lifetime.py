import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rgbd_worker_lifetime import worker_deadline


class WorkerLifetimeTests(unittest.TestCase):
    def test_legacy(self):
        self.assertEqual(worker_deadline(None,100),400)

    def test_session_survives_old_300_second_limit(self):
        deadline=worker_deadline(570,130)  # parent started at 100, budget 470
        self.assertGreater(deadline,130+300)
        self.assertEqual(deadline,570)

    def test_model_startup_does_not_renew_budget(self):
        self.assertEqual(worker_deadline(570,130),worker_deadline(570,200))

    def test_expired_deadline_not_extended(self):
        self.assertLess(worker_deadline(570,600),600)

    def test_invalid(self):
        for value in [0,-1,float('nan'),float('inf')]:
            with self.assertRaises(ValueError): worker_deadline(value,100)
