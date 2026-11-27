import os,unittest
from console_exit import ConsoleExit
class ConsoleExitTests(unittest.TestCase):
 def test_commands_nonblocking_and_eof(self):
  for command in ('q','exit','QUIT','退出'):
   read,write=os.pipe()
   with os.fdopen(read) as stream:
    c=ConsoleExit(stream);self.assertFalse(c.poll())
    os.write(write,(command+'\n').encode());self.assertTrue(c.poll())
    os.close(write);self.assertFalse(c.poll());self.assertFalse(c.enabled)
 def test_unrecognized_line_does_not_exit(self):
  read,write=os.pipe()
  with os.fdopen(read) as stream:
   c=ConsoleExit(stream);os.write(write,b'status\n');self.assertFalse(c.poll());os.close(write)
