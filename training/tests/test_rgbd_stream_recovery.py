"""Offline state-transition checks; not a substitute for live stream testing."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rgbd_localization import DiagnosticLocalization

class StreamRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.loc=DiagnosticLocalization(True)
        self.result={'detections':[dict(name='canned_juice',confidence=.99,
            core_pixels=100,valid_pixels=100,depth_p05_median_p95=[1.73,1.74,1.75],
            optical_surface_median_m=[.1,.2,1.74])],'class_conflicts':[]}
        self.tf=dict(translation=[0,0,0],rotation_xyzw=[0,0,0,1])

    def frame(self,stamp,now=None):
        return self.loc.evaluate(self.result,'canned_juice',int(stamp*1e9),int(stamp*1e9),
            int((stamp+.1 if now is None else now)*1e9),self.tf,'ready')

    def stabilize(self):
        for stamp in [1,1.2,1.4]: q=self.frame(stamp)
        self.assertTrue(q['stable'])

    def test_timeout_removes_coordinate_and_history(self):
        self.stabilize()
        q=self.loc.reject('waiting_for_fresh_result')
        self.assertFalse(q['stable']); self.assertNotIn('position_m',q)
        self.assertFalse(self.loc.gate.history)

    def test_recovery_requires_three_new_frames(self):
        self.stabilize(); self.loc.reject('waiting_for_fresh_result')
        self.assertEqual(self.frame(5)['count'],1)
        self.assertFalse(self.frame(5.2)['stable'])
        self.assertTrue(self.frame(5.4)['stable'])

    def test_delayed_inference_cannot_restore_stability(self):
        self.stabilize(); self.loc.reject('waiting_for_fresh_result')
        q=self.frame(1.6,now=5)
        self.assertEqual(q['reason'],'stale_or_future')
        self.assertFalse(self.loc.gate.history)

    def test_duplicate_after_recovery_not_counted(self):
        self.stabilize(); self.loc.reject('waiting_for_fresh_result')
        self.frame(5)
        self.assertEqual(self.frame(5)['reason'],'duplicate_or_reversed_time')
        self.assertEqual(self.frame(5.2)['count'],1)

    def test_long_gap_resets_even_without_timeout_callback(self):
        self.stabilize()
        self.assertEqual(self.frame(5)['count'],1)

if __name__=='__main__': unittest.main()
