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
 def test_public_scripts_start_first_and_forward_limit(self):
  source=Path(__file__).resolve().parents[2]/'launch'
  root_source=source if source.is_dir() else ROOT
  with tempfile.TemporaryDirectory() as out:
   root=Path(out)
   for mode in ('fixed','dynamic'):
    for f in root_source.glob(f'start_{mode}*.sh'):shutil.copy2(f,root/f.name)
   (root/'start_competition.sh').write_text('#!/bin/bash\nprintf "%s\\n" "$@"\n')
   for mode in ('fixed','dynamic'):
    suffix=['--dynamic-instructions'] if mode=='dynamic' else []
    for argv,n in [([],6),(['N:=0'],0),(['N:=2'],2),(['--N','6'],6),(['N:=4294967295'],4294967295)]:
     run=subprocess.run(['bash',str(root/f'start_{mode}.sh'),*argv],capture_output=True,text=True)
     self.assertEqual(run.returncode,0,run.stderr)
     self.assertEqual(run.stdout.splitlines(),['--execute','--from-session','1','--N',str(n)]+suffix)
    for argv in [['1'],['3'],['N:=-1'],['N:=1.5'],['N:=4294967296'],['N:='],['--N'],['--N','2','3']]:
     run=subprocess.run(['bash',str(root/f'start_{mode}.sh'),*argv],capture_output=True,text=True)
     self.assertEqual(run.returncode,2,(argv,run.stderr));self.assertEqual(run.stdout,'')
    run=subprocess.run(['bash',str(root/f'start_{mode}_session1.sh'),'N:=2'],capture_output=True,text=True)
    self.assertEqual(run.returncode,0,run.stderr)
    for n in range(2,7):
     run=subprocess.run(['bash',str(root/f'start_{mode}_session{n}.sh')],capture_output=True,text=True)
     self.assertEqual(run.returncode,2);self.assertEqual(run.stdout,'')
 def test_limit_defaults_and_uint32_validation(self):
  for mode in ([],['--dynamic-instructions']):
   self.assertEqual(parse_args(['--execute','--from-session','1',*mode]).N,6)
   for n in ('0','2','6','4294967295'):
    self.assertEqual(parse_args(['--execute','--from-session','1','--N',n,*mode]).N,int(n))
   for bad in ('-1','1.5','4294967296','oops'):
    with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
     parse_args(['--execute','--from-session','1','--N',bad,*mode])
