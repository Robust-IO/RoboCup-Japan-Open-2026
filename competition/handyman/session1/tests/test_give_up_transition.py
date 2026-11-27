import unittest
from itertools import permutations
from session1 import Session1
from dynamic_session import DynamicSession
from round_sequence import RoundSequence

class GiveUpTransitionTests(unittest.TestCase):
    def test_all_rounds_both_modes_both_handshake_orders(self):
        for dynamic in (False,True):
            for number in range(1,7):
                for order in permutations(('Environment','Are_you_ready?')):
                    with self.subTest(dynamic=dynamic,number=number,order=order):
                        seq=RoundSequence();seq.index=number-1
                        seq.message('Environment',seq.layouts[seq.index],1.)
                        seq.message('Are_you_ready?','',1.1)
                        seq.launch(seq.pending)
                        seq.message('Instruction','task',2.)
                        session=DynamicSession('s',profile={'layout':seq.layouts[seq.index]}) if dynamic else Session1('s')
                        session.state='navigate_destination';session.pending='op'
                        session.give_up('floor_target_not_found_after_retreat')
                        self.assertEqual([e['message'] for e in session.drain()],['Give_up'])
                        self.assertFalse(seq.finished);self.assertIsNone(seq.pending)
                        # Official failure ends this task, not the whole sequence.
                        session.moderator('Task_failed','Give_up',session_id='s')
                        seq.message('Task_failed','Give_up',3.)
                        self.assertEqual(session.state,'failed')
                        self.assertEqual(seq.finished,number==6)
                        seq.exited()  # real supervisor calls this after process retirement
                        self.assertEqual(seq.finished,number==6)
                        if number==6:continue
                        self.assertIsNone(seq.pending)
                        for i,name in enumerate(order):
                            seq.message(name,seq.layouts[number] if name=='Environment' else '',4.+i*.1)
                        self.assertEqual(seq.pending,number)
                        seq.launch(seq.pending)
                        self.assertEqual(seq.index,number)
                        self.assertFalse(seq.done);self.assertFalse(seq.finished)
    def test_new_handshake_can_arrive_before_old_process_exit(self):
        seq=RoundSequence();seq.index=4;seq.launch(4)
        seq.message('Instruction','task',1.)
        seq.message('Task_failed','Give_up',2.)
        seq.message('Are_you_ready?','',3.)
        seq.message('Environment','LayoutD',3.1)
        self.assertEqual(seq.pending,5)
        seq.exited();self.assertEqual(seq.pending,5)
        seq.launch(seq.pending);self.assertEqual(seq.index,5)
    def test_old_startup_handshake_does_not_skip_session5(self):
        seq=RoundSequence();seq.index=4;seq.launch(4)
        seq.message('Environment','LayoutD',1.)
        seq.message('Are_you_ready?','',1.1)
        self.assertIsNone(seq.pending)
        seq.message('Instruction','task',2.)
        seq.message('Task_failed','Give_up',3.)
        self.assertIsNone(seq.pending)
    def test_mission_complete_stops_sequence(self):
        seq=RoundSequence();seq.launch(0)
        seq.message('Mission_complete','',1.)
        self.assertTrue(seq.finished);self.assertIsNone(seq.pending)
