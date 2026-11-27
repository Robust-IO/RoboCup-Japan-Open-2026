import unittest
import io
from types import SimpleNamespace
from unittest.mock import Mock,patch
import tempfile
from pathlib import Path
from apple_worker_client import freshness,Worker


class Tests(unittest.TestCase):
    def test_fresh(self):self.assertTrue(freshness(1000000000,1200000000,10,10.1)['freshness_passed'])
    def test_old(self):self.assertFalse(freshness(1000000000,1600000000,10,10.1)['freshness_passed'])
    def test_future(self):self.assertFalse(freshness(1300000000,1200000000,10,10.1)['freshness_passed'])
    def test_slow_even_if_clock_pauses(self):self.assertFalse(freshness(1000000000,1200000000,10,11)['freshness_passed'])
    def test_not_authorization(self):self.assertFalse(freshness(1000000000,1200000000,10,10.1)['motion_authorized'])
    def test_inference_wait_can_service_ros_without_changing_timeout(self):
        worker=Worker.__new__(Worker)
        worker.selector=Mock();worker.selector.select.side_effect=[[],[],[(None,None)]]
        worker.p=SimpleNamespace(stdout=io.StringIO('{"status":"ready"}\n'))
        pump=Mock();self.assertEqual(worker.read(1,pump=pump)['status'],'ready')
        self.assertEqual(pump.call_count,2)
    def test_pumped_wait_still_times_out(self):
        worker=Worker.__new__(Worker);worker.selector=Mock();worker.selector.select.return_value=[]
        worker.p=SimpleNamespace(stdout=io.StringIO(''))
        with self.assertRaises(TimeoutError):worker.read(.001,pump=lambda:None)
    def test_roi_is_explicit_opt_in_and_propagated(self):
        with tempfile.TemporaryDirectory() as root:
            for enabled in (False,True):
                with patch('apple_worker_client.subprocess.Popen') as popen, \
                     patch('apple_worker_client.selectors.DefaultSelector'), \
                     patch.object(Worker,'read',return_value=dict(status='ready')):
                    popen.return_value.poll.return_value=0
                    worker=Worker('/python','registry',Path(root)/str(enabled),imgsz=960,confirm_apple_roi=enabled)
                    try:self.assertEqual('--confirm-apple-roi' in popen.call_args.args[0],enabled)
                    finally:worker.close()


if __name__=='__main__':unittest.main()
