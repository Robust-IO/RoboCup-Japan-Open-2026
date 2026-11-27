import unittest
from gripper_close_gate import CloseGate


class Tests(unittest.TestCase):
    def feed(self, opening, initial=.65):
        gate=CloseGate(initial);result=None
        for i in range(9):result=gate.observe(6+i*.1,10_000_000_000+i*100_000_000,opening,0)
        return result

    def test_apple_contact_not_success(self):
        r=self.feed(.4122018066)
        self.assertEqual(r['state'],'contact_candidate')
        self.assertFalse(r['grasp_success_verified'])
        self.assertTrue(r['requires_visual_lift_verification'])

    def test_empty_close_not_success(self):
        self.assertEqual(self.feed(0)['state'],'closed_without_contact_evidence')

    def test_unchanged_is_not_contact(self):
        self.assertEqual(self.feed(.65)['state'],'closure_inconclusive')

    def test_small_motion_is_inconclusive(self):
        self.assertEqual(self.feed(.63)['state'],'closure_inconclusive')

    def test_before_deadline(self):
        g=CloseGate(.65)
        for i in range(50):self.assertIsNone(g.observe(i*.1,i*100_000_000,.412,0))

    def test_duplicate_stamp(self):
        g=CloseGate(.65)
        for i in range(20):self.assertIsNone(g.observe(6+i*.1,123,.412,0))

    def test_gap_resets(self):
        g=CloseGate(.65)
        for i in range(5):g.observe(6+i*.1,i*100_000_000,.412,0)
        self.assertIsNone(g.observe(7,1_000_000_000,.412,0))
        self.assertEqual(len(g.rows),1)

    def test_explicit_reset(self):
        g=CloseGate(.65)
        for i in range(5):g.observe(6+i*.1,i*100_000_000,.412,0)
        g.reset_window()
        self.assertIsNone(g.observe(6.5,500_000_000,.412,0))

    def test_oscillation(self):
        g=CloseGate(.65)
        for i in range(20):self.assertIsNone(g.observe(6+i*.1,i*100_000_000,.41+.02*(i%2),0))

    def test_bad_feedback(self):
        for value in (float('nan'),-1,2):
            with self.assertRaises(ValueError):CloseGate(.65).observe(6,1,value,0)
        with self.assertRaises(ValueError):CloseGate(.65).observe(6,1,.4,.02)

    def test_reversed_clock(self):
        g=CloseGate(.65);g.observe(6,100,.4,0)
        with self.assertRaises(ValueError):g.observe(6.1,99,.4,0)


if __name__=='__main__':unittest.main()
