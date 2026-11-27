import unittest,tempfile,os,socket,json,ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import resident_inference as resident
class ShortIPC(unittest.TestCase):
 def test_long_package_tmp_uses_real_short_bind(self):
  with tempfile.TemporaryDirectory(prefix='hm-ipc-',dir='/tmp') as tmp:
   registry=Path(tmp)/'registry.json';registry.write_text('{"imgsz":960}')
   def ready(endpoint,*a,**kw):
    self.assertLessEqual(len(os.fsencode(endpoint)),107)
    self.assertEqual(Path(endpoint).parent,Path(tmp))
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:s.bind(str(endpoint))
   with patch.dict(os.environ,{'TMPDIR':'/verylong/'+'x'*200,'HANDYMAN_IPC_DIR':tmp}),patch.object(resident,'service_key',return_value='f'*24),patch.object(resident,'rpc',side_effect=ready),patch.object(resident,'SharedWorker',return_value=SimpleNamespace(ready={})):
    self.assertTrue(resident.connect('unused',registry).ready['resident_reused'])
 def test_startup_retry_branch_retains_round_and_archives_failed_attempt(self):
  from round_sequence import RoundSequence
  source=(Path(__file__).parents[1]/'run_sessions12.py').read_text();tree=ast.parse(source)
  node=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and 'startup_failures.get' in ast.unparse(n) and ast.unparse(n.test).startswith('code != 0'))
  with tempfile.TemporaryDirectory() as tmp:
   rd=Path(tmp)/'session3';rd.mkdir();seq=RoundSequence();seq.index=2;seq.started=True
   env=dict(code=1,seq=seq,startup_failures={},round_dir=rd,record=lambda **kw:None)
   exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual startup retry>','exec'),env)
   self.assertFalse(seq.started);self.assertFalse(seq.done);self.assertEqual(seq.index,2)
   self.assertTrue((Path(tmp)/'session3-startup-failed-1').exists())
 def test_startup_retry_has_explicit_terminal_error(self):
  from round_sequence import RoundSequence
  source=(Path(__file__).parents[1]/'run_sessions12.py').read_text();tree=ast.parse(source)
  node=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and 'startup_failures.get' in ast.unparse(n) and ast.unparse(n.test).startswith('code != 0'))
  seq=RoundSequence();seq.started=True
  with self.assertRaisesRegex(RuntimeError,'round_initialization_failed_after_3_attempts'):
   exec(compile(ast.Module(body=[node],type_ignores=[]),'<retry>','exec'),dict(code=1,seq=seq,startup_failures={0:2},round_dir=Path('/unused'),record=lambda **kw:None))
