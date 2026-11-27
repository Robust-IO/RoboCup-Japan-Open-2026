import unittest,tempfile,threading
from pathlib import Path
from unittest.mock import patch
from startup_parallel import prepare_with_model
class ParallelTests(unittest.TestCase):
 def test_model_and_hand_overlap(self):
  started=threading.Event();hand=threading.Event();model=object()
  def factory():
   started.set();assert hand.wait(2);return model
  def prepare(rt):
   assert started.wait(2);hand.set();return {'motion_sent':True}
  with tempfile.TemporaryDirectory() as d,patch('startup_parallel.prepare',prepare):
   self.assertIs(prepare_with_model(factory,None,Path(d)),model)
 def test_hand_failure_closes_model(self):
  from unittest.mock import Mock
  model=Mock()
  with tempfile.TemporaryDirectory() as d,patch('startup_parallel.prepare',side_effect=ValueError('bad_feedback')):
   with self.assertRaisesRegex(ValueError,'bad_feedback'):prepare_with_model(lambda:model,None,Path(d))
  model.close.assert_called_once()
