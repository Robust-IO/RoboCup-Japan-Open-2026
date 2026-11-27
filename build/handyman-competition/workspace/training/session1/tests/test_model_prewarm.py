import unittest,tempfile,json,subprocess
from pathlib import Path
from unittest.mock import Mock,patch
from model_prewarm import command,start,stop_waiter
class PrewarmTests(unittest.TestCase):
 def test_command_matches_task_model(self):
  with tempfile.TemporaryDirectory() as td:
   p=Path(td)/'config.json';p.write_text(json.dumps({'model_python':'/pack/python','registry':'/pack/model.json'}))
   cmd=command(p,'/pack')
   self.assertEqual(cmd[1],'/pack/tools/apple-transfer-auto-v1/resident_inference.py')
   self.assertEqual(cmd[-2:],['--imgsz','960'])
   self.assertIn('/pack/model.json',cmd)
 def test_completed_waiter_is_not_killed(self):
  child=Mock();child.poll.return_value=0;stop_waiter(child);child.terminate.assert_not_called()
 def test_waiter_shutdown_is_bounded(self):
  child=Mock();child.poll.return_value=None
  child.wait.side_effect=[subprocess.TimeoutExpired('worker',3),0]
  stop_waiter(child);child.terminate.assert_called_once();child.kill.assert_called_once()
