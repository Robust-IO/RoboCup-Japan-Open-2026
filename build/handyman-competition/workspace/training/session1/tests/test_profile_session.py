import json
from pathlib import Path
import unittest
from session1 import Session1
class ProfileSessionTests(unittest.TestCase):
    def test_session2_protocol_and_room_qualified_target(self):
        p=json.loads((Path(__file__).resolve().parents[1]/'profiles/session2.json').read_text())
        s=Session1('s2',profile=p);s.preflight(ready=True)
        s.moderator('Environment','LayoutB',session_id='s2');s.moderator('Are_you_ready?',session_id='s2')
        self.assertEqual(s.state,'waiting_instruction')
        s.moderator('Instruction',p['expected_instruction'],session_id='s2')
        self.assertEqual(s.state,'navigate_room');self.assertEqual(s.task.target,'canned_juice')
        self.assertEqual(s.task.source_table,'wooden_side_table#living_room')
        self.assertEqual(s.task.destination,'trash_box_for_bottle_can#living_room')
    def test_wrong_layout_not_executed(self):
        p=json.loads((Path(__file__).resolve().parents[1]/'profiles/session2.json').read_text())
        s=Session1('s2',profile=p);s.moderator('Environment','LayoutA',session_id='s2')
        self.assertEqual(s.state,'stopped')
