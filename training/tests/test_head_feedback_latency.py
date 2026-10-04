import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from head_view_trial import Settling,JOINTS,SEARCH_FEEDBACK_MAX_S
from search_head_stage import StopConfirmation


class LatencyTests(unittest.TestCase):
    def test_330ms_feedback_can_settle(self):
        gate=Settling((0.,-.25),SEARCH_FEEDBACK_MAX_S)
        for i in range(10):
            stamp=1_000_000_000+i*150_000_000
            ok,_=gate.sample(JOINTS,(0.,-.25),stamp,stamp+330_000_000,i*.15)
        self.assertTrue(ok)

    def test_delayed_post_hold_feedback_confirms(self):
        gate=StopConfirmation((0.,-.25),1_000_000_000,0)
        for i in range(1,8):
            stamp=1_000_000_000+i*150_000_000
            gate.sample(JOINTS,(0.,-.25),stamp,stamp+330_000_000,i*.15+.33)
        self.assertTrue(gate.verified(1.4))
        self.assertFalse(gate.verified(2.))

    def test_over_limit_still_rejected(self):
        gate=Settling((0.,-.25),SEARCH_FEEDBACK_MAX_S)
        ok,pose=gate.sample(JOINTS,(0.,-.25),1_000_000_000,1_501_000_000,0)
        self.assertFalse(ok);self.assertIsNone(pose)

    def test_legacy_trial_unchanged(self):
        ok,pose=Settling((0.,-.25)).sample(JOINTS,(0.,-.25),1_000_000_000,1_330_000_000,0)
        self.assertFalse(ok);self.assertIsNone(pose)

    def test_limit_is_bounded(self):
        for limit in [0,.6,float('nan')]:
            with self.assertRaises(ValueError):Settling((0.,-.25),limit)
