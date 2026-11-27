import unittest,tempfile,shutil,subprocess,json,contextlib,io
from pathlib import Path
from run_sessions12 import parse_args,initial_index,round_spec
from round_sequence import RoundSequence
ROOT=Path(__file__).resolve().parents[4]
class AnySessionTests(unittest.TestCase):
 def test_all_modes_all_starts(self):
  for dynamic in (False,True):
   for n in range(1,7):
    args=parse_args(['--execute','--from-session',str(n)]+(['--dynamic-instructions'] if dynamic else []))
    self.assertEqual(initial_index(args),n-1);self.assertEqual(args.dynamic_instructions,dynamic)
    seq=RoundSequence();seq.index=initial_index(args)
    for i in range(n-1,6):
     seq.message('Are_you_ready?','',i*10);seq.message('Environment',round_spec(i)['layout'],i*10+.1)
     self.assertEqual(seq.pending,i);seq.launch(i)
     seq.message('Instruction','task',i*10+.2)
     seq.exited()
    self.assertTrue(seq.finished)
 def test_legacy_flags(self):
  for n in range(1,7):self.assertEqual(initial_index(parse_args(['--execute',f'--from-session{n}-initial'])),n-1)
  self.assertEqual(initial_index(parse_args(['--execute','--resume-session2-initial'])),1)
 def test_invalid_arguments(self):
  for argv in [['--execute','--from-session','7'],['--execute','--from-session','1','--from-session3-initial'],['--execute','--dynamic-instructions','--resume-session3-search-from','/tmp/x']]:
   with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):parse_args(argv)
 def test_twelve_scripts_without_running_robot(self):
  with tempfile.TemporaryDirectory() as out:
   root=Path(out)
   for mode in ('fixed','dynamic'):
    for f in ROOT.glob(f'start_{mode}*.sh'):shutil.copy2(f,root/f.name)
   (root/'start_competition.sh').write_text('#!/bin/bash\nprintf "%s\\n" "$@"\n')
   for mode in ('fixed','dynamic'):
    for n in range(1,7):
     p=subprocess.run(['bash',str(root/f'start_{mode}_session{n}.sh')],capture_output=True,text=True)
     self.assertEqual(p.returncode,0,p.stderr)
     self.assertEqual(p.stdout.splitlines(),['--execute','--from-session',str(n)]+(['--dynamic-instructions'] if mode=='dynamic' else []))
    for bad in ('0','7','oops'):
     p=subprocess.run(['bash',str(root/f'start_{mode}.sh'),bad],capture_output=True,text=True)
     self.assertEqual(p.returncode,2);self.assertEqual(p.stdout,'')
