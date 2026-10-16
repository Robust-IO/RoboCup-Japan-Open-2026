import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tf_motion_gate import TfMotionGate,TfSearchArrival
from search_scan import SearchScan
from search_arrival import SearchArrival


class MotionTests(unittest.TestCase):
    def setUp(self):self.g=TfMotionGate()
    def feed(self,i,x=0,yaw=0,**extra):
        return self.g.update(dict(x=x,y=0,yaw=yaw),1_000_000_000+i*100_000_000,
            1_000_000_000+i*100_000_000,i*.1,**extra)
    def test_stationary(self):
        for i in range(12):r=self.feed(i)
        self.assertTrue(r['stationary']);self.assertFalse(r['actionable'])
    def test_noise(self):
        for i in range(12):r=self.feed(i,x=.0006*(-1)**i)
        self.assertTrue(r['stationary'])
    def test_callback_timing_jitter_keeps_complete_window(self):
        for i in range(40):
            ns=1_000_000_000+i*100_000_000
            now=i*.1+(.001 if i%3==0 else 0)
            r=self.g.update(dict(x=0,y=0,yaw=0),ns,ns,now)
            if i>=12:self.assertTrue(r['stationary'],r)
    def test_slow_drift(self):
        for i in range(12):r=self.feed(i,x=.001*i)
        self.assertFalse(r['stationary']);self.assertEqual(r['reason'],'motion_detected')
    def test_out_and_back(self):
        for i in range(11):r=self.feed(i,x=.01*math.sin(i*math.pi/10))
        self.assertFalse(r['stationary'])
    def test_rotation(self):
        for i in range(12):r=self.feed(i,yaw=.001*i)
        self.assertFalse(r['stationary'])
    def test_wraparound(self):
        for i in range(12):r=self.feed(i,yaw=(math.pi-.0001) if i%2 else (-math.pi+.0001))
        self.assertTrue(r['stationary'])
    def test_duplicate(self):
        self.feed(1);r=self.feed(1)
        self.assertFalse(r['valid']);self.assertEqual(len(self.g.samples),0)
    def test_gap(self):
        for i in range(12):self.feed(i)
        self.assertFalse(self.feed(20)['valid'])
    def test_wrong_frame(self):self.assertFalse(self.feed(1,parent='map')['valid'])
    def test_nan(self):self.assertFalse(self.feed(1,x=float('nan'))['valid'])
    def test_future(self):
        self.assertFalse(self.g.update(dict(x=0,y=0,yaw=0),2_000_000_000,1_000_000_000,0)['valid'])
    def test_arrival_requires_goal_and_map(self):
        scan=SearchScan('test','canned_juice',[dict(x=0,y=0,yaw=0)],0)
        gate=SearchArrival(scan,'goal');adapter=TfSearchArrival(gate)
        pose=dict(x=0,y=0,yaw=0)
        for i in range(25):
            ns=1_000_000_000+i*100_000_000
            adapter.sample(pose,pose,ns,ns,ns,i*.1)
        self.assertEqual(scan.phase,'navigate')
        gate.result('goal',True,3_400_000_000)
        for i in range(25,38):
            ns=1_000_000_000+i*100_000_000
            adapter.sample(pose,None,ns,ns,ns,i*.1)
        self.assertEqual(scan.phase,'navigate')
        for i in range(38,51):
            ns=1_000_000_000+i*100_000_000
            adapter.sample(pose,pose,ns,ns,ns,i*.1)
        self.assertEqual(scan.phase,'observe')

if __name__=='__main__':unittest.main()
