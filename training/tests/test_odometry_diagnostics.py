import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from tf_derived_odometry import OdometryDiagnostics


def sample(i, vx):
    return dict(valid=True, reason='estimated', stamp_ns=1_000_000_000+i*100_000_000,
                twist=dict(vx=vx, vy=0., wz=0.))


class DiagnosticsTests(unittest.TestCase):
    def test_raw_unchanged(self):
        d=OdometryDiagnostics(); row=sample(0,.01)
        self.assertEqual(d.annotate(row)['twist'],row['twist'])
        self.assertNotIn('health',row)
    def test_jitter_reduced(self):
        d=OdometryDiagnostics()
        for i in range(20): r=d.annotate(sample(i,.008*(-1)**i))
        self.assertLess(abs(r['diagnostic_filtered_twist']['vx']),.003)
    def test_slow_motion_not_zeroed(self):
        d=OdometryDiagnostics()
        for i in range(20): r=d.annotate(sample(i,.001))
        self.assertAlmostEqual(r['diagnostic_filtered_twist']['vx'],.001)
        self.assertFalse(r['health']['stop_confirmed'])
    def test_fault_resets_filter(self):
        d=OdometryDiagnostics();d.annotate(sample(0,.2))
        r=d.annotate(dict(valid=False,reason='receive_timeout_or_clock_reset'))
        self.assertEqual(r['health']['state'],'unavailable')
        self.assertNotIn('diagnostic_filtered_twist',r)
        self.assertEqual(d.annotate(sample(8,-.1))['diagnostic_filtered_twist']['vx'],-.1)
    def test_states_and_sequence(self):
        d=OdometryDiagnostics()
        for i,(reason,state) in enumerate([('warming_up','warming'),('gap_rewarming','warming'),
                                         ('multiple_or_unknown_tf_publishers','unavailable'),('probe_stopped','stopped')],1):
            r=d.annotate(dict(valid=False,reason=reason))
            self.assertEqual(r['health']['state'],state)
            self.assertEqual(r['health']['sequence'],i)
            self.assertFalse(r['health']['navigation_authorized'])
    def test_none(self):
        self.assertIsNone(OdometryDiagnostics().annotate(None))
    def test_step_response_is_delayed(self):
        d=OdometryDiagnostics();d.annotate(sample(0,0.))
        r=d.annotate(sample(1,.2))
        self.assertGreater(r['diagnostic_filtered_twist']['vx'],0.)
        self.assertLess(r['diagnostic_filtered_twist']['vx'],.2)


if __name__=='__main__':unittest.main()
