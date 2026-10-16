"""Recorded receive cadence with synthetic stationary poses (not pose replay)."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tf_motion_gate import TfMotionGate,TfSearchObserver
from search_scan import SearchScan
from search_arrival import SearchArrival


class CadenceTests(unittest.TestCase):
    def build(self):
        self.pose=dict(x=0,y=0,yaw=0)
        self.scan=SearchScan('t','can',[self.pose],0,timeout_s=30,view_s=10)
        arrival=SearchArrival(self.scan,'g');arrival.result('g',True,1)
        self.observer=TfSearchObserver(arrival,0,recover_transient_tf=True,max_gap_s=.5)

    def sample(self,t,pose=None):
        ns=1_000_000_000+round(t*1e9)
        return self.observer.sample(pose or self.pose,self.pose,ns,ns,t)

    def test_recorded_cadence_can_settle(self):
        self.build()
        times=[10189.450786730,10189.776684146,10189.957776758,
               10190.356458499,10190.774710240,10191.055568912,
               10191.271473262,10191.588817068,10191.892124423,
               10192.229986495,10192.420063618,10192.672902398,
               10192.978190660,10193.177516031]
        for t in times:self.sample(t-times[0])
        self.assertEqual(self.scan.phase,'observe')
        self.assertFalse(self.observer.recovery_used)

    def test_three_hz_keeps_six_samples(self):
        gate=TfMotionGate(.5)
        for i in range(15):
            ns=1_000_000_000+i*333_333_333
            result=gate.update(dict(x=0,y=0,yaw=0),ns,ns,i/3)
        self.assertTrue(result['stationary'])
        self.assertGreaterEqual(result['count'],6)

    def test_long_gap_requires_new_arrival(self):
        self.build()
        for i in range(12):self.sample(i*.3)
        self.assertEqual(self.scan.phase,'observe')
        self.observer.tick(3.9)
        self.assertEqual(self.scan.phase,'navigate')
        self.assertIsNone(self.observer.arrival.adapter)
        deadline=self.observer.recovery_deadline
        self.observer.tick(deadline+.01)
        self.assertEqual(self.scan.phase,'incomplete')

    def test_motion_still_aborts(self):
        self.build()
        for i in range(12):self.sample(i*.3)
        self.sample(3.6,dict(x=.03,y=0,yaw=0))
        self.assertEqual(self.scan.phase,'incomplete')

    def test_stale_and_duplicate_still_rejected(self):
        gate=TfMotionGate(.5);pose=dict(x=0,y=0,yaw=0)
        self.assertFalse(gate.update(pose,1_000_000_000,1_600_000_000,0)['valid'])
        gate.update(pose,2_000_000_000,2_000_000_000,1)
        self.assertFalse(gate.update(pose,2_000_000_000,2_000_000_000,1.1)['valid'])

    def test_gap_limit_cannot_exceed_freshness(self):
        for limit in [0,.51,float('nan')]:
            with self.assertRaises(ValueError):TfMotionGate(limit)
