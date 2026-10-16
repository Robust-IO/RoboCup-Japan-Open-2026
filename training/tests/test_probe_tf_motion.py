import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from probe_tf_motion import MotionObserver,planar_pose


class ProbeTests(unittest.TestCase):
    def test_yaw(self):
        tf=S(translation=S(x=1,y=2,z=0),rotation=S(x=0,y=0,z=math.sin(.4),w=math.cos(.4)))
        self.assertAlmostEqual(planar_pose(tf)['yaw'],.8)
        tf.rotation.w=0
        with self.assertRaises(ValueError):planar_pose(tf)

    def test_nonfinite(self):
        tf=S(translation=S(x=0,y=0,z=float('nan')),rotation=S(x=0,y=0,z=0,w=1))
        with self.assertRaises(ValueError):planar_pose(tf)

    def test_timeout_and_recovery(self):
        o=MotionObserver(0)
        for i in range(12):
            ns=1_000_000_000+i*100_000_000
            row=o.receive(dict(x=0,y=0,yaw=0),ns,ns,i*.1)
        self.assertTrue(row['stationary'])
        self.assertFalse(o.tick(1.5)['stationary'])
        self.assertIsNone(o.tick(1.6))
        self.assertEqual(len(o.gate.samples),0)
        self.assertFalse(o.receive(dict(x=0,y=0,yaw=0),3_000_000_000,3_000_000_000,2)['stationary'])

    def test_no_input_timeout(self):
        o=MotionObserver(0)
        self.assertEqual(o.tick(.31)['reason'],'tf_stream_timeout')

if __name__=='__main__':unittest.main()
