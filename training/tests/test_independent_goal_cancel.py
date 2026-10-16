import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from independent_goal_cancel import ExactCancelEvidence


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.now=0.
        self.gate=ExactCancelEvidence('a'*32,clock=lambda:self.now)

    def test_acceptance_alone_not_terminal(self):
        self.gate.acknowledgement(0,['a'*32]);self.assertEqual(self.gate.poll(),'waiting')
        self.gate.terminal(5);self.assertEqual(self.gate.poll(),'goal_cancel_confirmed')

    def test_terminal_before_ack(self):
        self.gate.terminal(5);self.assertEqual(self.gate.poll(),'waiting')
        self.gate.acknowledgement(0,['a'*32]);self.assertEqual(self.gate.poll(),'goal_cancel_confirmed')

    def test_wrong_uuid_and_rejection_sticky(self):
        for code,ids in ((0,['b'*32]),(1,['a'*32])):
            self.setUp();self.gate.acknowledgement(code,ids)
            self.gate.terminal(5);self.assertEqual(self.gate.poll(),'rejected_or_uuid_mismatch')

    def test_timeout_sticky(self):
        self.now=2.;self.gate.acknowledgement(0,['a'*32]);self.gate.terminal(5)
        self.assertEqual(self.gate.poll(),'timeout')

    def test_other_result_not_success(self):
        for status in (0,4,6):
            self.setUp();self.gate.acknowledgement(0,['a'*32]);self.gate.terminal(status)
            self.assertEqual(self.gate.poll(),'not_cancelled_terminal')

    def test_zero_or_invalid_uuid_refused(self):
        for value in ('0'*32,'bad','A'*32):
            with self.assertRaises(ValueError):ExactCancelEvidence(value)


if __name__=='__main__':unittest.main()
