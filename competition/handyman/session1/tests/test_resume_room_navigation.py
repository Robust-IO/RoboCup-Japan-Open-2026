import json,math,tempfile,unittest
from pathlib import Path
from session1 import Session1
from resume_room_navigation import restore
class RoomResumeTests(unittest.TestCase):
 def test_same_round_without_handshake_replay(self):
  with tempfile.TemporaryDirectory() as t:
   p=Path(t);s=Session1('new');old=s.snapshot();old.update(state='stopped',evidence={})
   (p/'live-state.json').write_text(json.dumps(old));(p/'stop-confirmation.json').write_text(json.dumps(dict(motion_stopped=True)))
   (p/'resume-protocol.log').write_text('Moderator: Instruction '+s.instruction)
   result=restore(s,p,dict(joints=[0,0,0,-math.pi/2,0,.65]));self.assertFalse(result['handshake_replayed']);self.assertEqual(s.state,'navigate_room')
   self.assertEqual([e['kind'] for e in s.drain()],['operation'])
 def test_terminal_protocol_rejected(self):
  with tempfile.TemporaryDirectory() as t:
   p=Path(t);s=Session1('new');old=s.snapshot();old.update(state='stopped',evidence={})
   (p/'live-state.json').write_text(json.dumps(old));(p/'stop-confirmation.json').write_text(json.dumps(dict(motion_stopped=True)))
   (p/'resume-protocol.log').write_text('Moderator: Instruction '+s.instruction+'\nModerator: Task_failed')
   with self.assertRaises(ValueError):restore(s,p,dict(joints=[0,0,0,-math.pi/2,0,.65]))
