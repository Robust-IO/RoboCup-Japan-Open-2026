import sys
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
import subprocess
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rgbd_shutdown import cleanup_steps,stop_worker

class ShutdownTests(unittest.TestCase):
    def test_failure_does_not_skip_later_steps(self):
        calls=[]
        def fail(): raise RuntimeError('context invalid')
        errors=cleanup_steps([('publish',fail),('worker',lambda:calls.append('worker')),('ros',lambda:calls.append('ros'))])
        self.assertEqual(calls,['worker','ros']); self.assertEqual(errors,[('publish','context invalid')])
    def test_none_worker(self): stop_worker(None)
    @patch('rgbd_shutdown.os.killpg',side_effect=ProcessLookupError)
    def test_already_exited(self,kill):
        worker=Mock(pid=321); stop_worker(worker); kill.assert_called_once()
    @patch('rgbd_shutdown.os.killpg')
    def test_stuck_worker(self,kill):
        worker=Mock(pid=321)
        worker.wait.side_effect=[subprocess.TimeoutExpired('worker',3),subprocess.TimeoutExpired('worker',3),0,0]
        stop_worker(worker)
        self.assertEqual(kill.call_count,2)
    def test_all_errors_collected(self):
        def fail(): raise OSError('disk')
        self.assertEqual(len(cleanup_steps([('a',fail),('b',fail)])),2)

if __name__=='__main__': unittest.main()
