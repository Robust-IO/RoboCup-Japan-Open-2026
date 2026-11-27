import unittest,tempfile
from pathlib import Path
from types import SimpleNamespace
from motion_stop import StopController,MotionGate
from application import WorkerPort
from concurrent.futures import Future

class TerminalTests(unittest.TestCase):
 def controller(self,d):
  gate=MotionGate();gate.cancel()
  c=StopController(SimpleNamespace(state=lambda:self.fail('must not wait for stale feedback')),
      SimpleNamespace(active_child=None),gate,Path(d))
  c.application=SimpleNamespace(session=SimpleNamespace(session_id='s'),
      official_simulation_end=dict(session_id='s',message='Task_failed',detail='Time_is_up'))
  return c
 def test_official_end_retires_without_claiming_measured_stop(self):
  with tempfile.TemporaryDirectory() as d:
   c=self.controller(d)
   try:
    proof=c._verify({},False)
    self.assertTrue(proof['commands_retired']);self.assertFalse(proof['motion_stopped'])
    future=Future();future.set_result(dict(status='cancelled'));stop=Future();stop.set_result(proof)
    handle=dict(future=future,stop_future=stop,stop_required=True)
    self.assertEqual(WorkerPort(None,None,None).poll(handle)['status'],'cancelled')
   finally:c.close()
 def test_running_child_must_retire_first(self):
  with tempfile.TemporaryDirectory() as d:
   c=self.controller(d);c.motion.active_child=SimpleNamespace(poll=lambda:None)
   try:
    with self.assertRaisesRegex(RuntimeError,'finite_child_still_running'):c._verify({},False)
   finally:c.close()
 def test_unconfirmed_stop_not_accepted(self):
  f=Future();f.set_result(dict(status='cancelled'));stop=Future();stop.set_result(dict(motion_stopped=False))
  with self.assertRaisesRegex(RuntimeError,'motion_stop_not_confirmed'):
   WorkerPort(None,None,None).poll(dict(future=f,stop_future=stop,stop_required=True))
