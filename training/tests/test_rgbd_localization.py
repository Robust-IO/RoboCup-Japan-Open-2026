import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rgbd_localization import DiagnosticLocalization,load_audit

def result(**changes):
    d=dict(name='canned_juice',confidence=.99,core_pixels=100,valid_pixels=100,
           depth_p05_median_p95=[1.73,1.74,1.75],optical_surface_median_m=[.1,.2,1.74])
    d.update(changes)
    return dict(detections=[d],class_conflicts=[])

class LocalizationTests(unittest.TestCase):
    def run_frame(self,loc,i=1,r=None,status='ready'):
        return loc.evaluate(result() if r is None else r,'canned_juice',i*10**9,i*10**9,i*10**9+10**8,
            dict(translation=[0,0,0],rotation_xyzw=[.5,-.5,.5,-.5]),status)
    def test_three_frames_in_odom(self):
        loc=DiagnosticLocalization(True)
        for i in range(1,4): q=self.run_frame(loc,i)
        self.assertTrue(q['stable']); self.assertEqual(q['frame_id'],'odom')
        for a,b in zip(q['position_m'],[1.74,-.1,-.2]): self.assertAlmostEqual(a,b)
    def test_default_unverified(self):
        self.assertEqual(self.run_frame(DiagnosticLocalization())['reason'],'unverified_geometry')
        self.assertFalse(load_audit(None))
    def test_absence_resets(self):
        loc=DiagnosticLocalization(True); self.run_frame(loc)
        self.assertFalse(self.run_frame(loc,2,dict(detections=[]))['stable'])
        self.assertEqual(self.run_frame(loc,3)['count'],1)
    def test_invalid_quality(self):
        for changes in [dict(confidence=.3),dict(valid_pixels=0),dict(depth_p05_median_p95=[.3,.31,.32]),
                        dict(depth_p05_median_p95=[1.,1.5,2.]),dict(optical_surface_median_m=[float('nan'),0,1])]:
            self.assertFalse(self.run_frame(DiagnosticLocalization(True),r=result(**changes))['stable'])
    def test_tf_timeout(self):
        self.assertEqual(self.run_frame(DiagnosticLocalization(True),status='timeout')['reason'],'missing_or_expired_tf')
    def test_duplicate(self):
        loc=DiagnosticLocalization(True); self.run_frame(loc)
        self.assertEqual(self.run_frame(loc)['reason'],'duplicate_or_reversed_time')

if __name__=='__main__': unittest.main()
