import ast,unittest,tempfile,subprocess
from pathlib import Path
from types import SimpleNamespace
from round_sequence import RoundSequence
class PreparationRetryTests(unittest.TestCase):
 def test_timeout_rearms_same_round_without_task_launch(self):
  tree=ast.parse((Path(__file__).parents[1]/'run_sessions12.py').read_text())
  handler=next(n for n in ast.walk(tree) if isinstance(n,ast.ExceptHandler) and 'round_preparation_retry_waiting_fresh_handshake' in ast.unparse(n))
  with tempfile.TemporaryDirectory() as tmp:
   rd=Path(tmp)/'session3';rd.mkdir();seq=RoundSequence();seq.index=2;seq.started=True
   env=dict(seq=seq,exc=subprocess.TimeoutExpired('clock',40),subprocess=subprocess,preparation_failures={},round_dir=rd,record=lambda **kw:None,future=object(),out=Path(tmp))
   code=ast.Module(body=[ast.For(target=ast.Name(id='_',ctx=ast.Store()),iter=ast.List(elts=[ast.Constant(1)],ctx=ast.Load()),body=handler.body,orelse=[])],type_ignores=[])
   exec(compile(ast.fix_missing_locations(code),'<real prepare retry>','exec'),env)
   self.assertEqual(seq.index,2);self.assertFalse(seq.started);self.assertIsNone(env['future'])
   self.assertTrue((Path(tmp)/'session3-preparation-failed-1').exists())
 def test_future_feedback_is_not_accepted_without_source_sync(self):
  from startup_clock_ready import ready_window
  rows=[(i*.1,1000000000+i*100000000,-.003-.038*i/29) for i in range(30)]
  self.assertFalse(ready_window(rows,3.))
  self.assertTrue(ready_window([(t,s,a+.056) for t,s,a in rows],3.))
