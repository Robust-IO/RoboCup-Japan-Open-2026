import unittest
from unittest.mock import patch
import run_sessions12 as runner
class RetirementTests(unittest.TestCase):
 def test_zombie_is_not_running(self):
  with patch('pathlib.Path.read_text',return_value='42 (node) Z 1 42 0'):
   self.assertFalse(runner.process_running(42))
 def test_sleeping_node_is_running(self):
  with patch('pathlib.Path.read_text',return_value='42 (node) S 1 42 0'):
   self.assertTrue(runner.process_running(42))
 def test_exited_node_is_not_running(self):
  with patch('pathlib.Path.read_text',side_effect=FileNotFoundError):
   self.assertFalse(runner.process_running(42))
