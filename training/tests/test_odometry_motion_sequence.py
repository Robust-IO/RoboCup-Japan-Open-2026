"""Synthetic motion transitions, not evidence of actual Unity motion accuracy."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tf_derived_odometry import TfDerivedOdometry


class MotionSequenceTests(unittest.TestCase):
    def run_sequence(self,velocities):
        e=TfDerivedOdometry();x=y=yaw=0.;rows=[]
        for i,(vx,wz) in enumerate(velocities):
            x+=vx*math.cos(yaw)*.1;y+=vx*math.sin(yaw)*.1;yaw+=wz*.1
            stamp=1_000_000_000+i*100_000_000
            rows.append(e.update(dict(x=x,y=y,yaw=yaw),stamp,stamp,i*.1))
        return rows
    def test_forward_then_stop(self):
        rows=self.run_sequence([(0.,0.)]*6+[(.05,0.)]*10+[(0.,0.)]*8)
        self.assertAlmostEqual(rows[15]['twist']['vx'],.05)
        self.assertAlmostEqual(rows[18]['twist']['vx'],0.)
        self.assertGreater(rows[16]['twist']['vx'],0.) # finite-window delay is expected
    def test_both_turn_directions(self):
        for direction in (-1,1):
            rows=self.run_sequence([(0.,direction*.1)]*10+[(0.,0.)]*6)
            self.assertAlmostEqual(rows[9]['twist']['wz'],direction*.1)
            self.assertAlmostEqual(rows[-1]['twist']['wz'],0.)
    def test_reverse(self):
        rows=self.run_sequence([(-.05,0.)]*10)
        self.assertAlmostEqual(rows[-1]['twist']['vx'],-.05)
    def test_turn_then_forward_body_frame(self):
        rows=self.run_sequence([(0.,.1)]*30+[(.05,0.)]*10)
        self.assertAlmostEqual(rows[-1]['twist']['vx'],.05)
        self.assertAlmostEqual(rows[-1]['twist']['vy'],0.)


if __name__=='__main__':unittest.main()
