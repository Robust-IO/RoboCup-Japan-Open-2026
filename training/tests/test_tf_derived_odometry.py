import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tf_derived_odometry import TfDerivedOdometry
from probe_derived_odometry import planar_transform


class DerivedTests(unittest.TestCase):
    def setUp(self):self.e=TfDerivedOdometry()
    def feed(self,i,x=0.,y=0.,yaw=0.,now=None):
        stamp=1_000_000_000+round(i*100_000_000)
        return self.e.update(dict(x=x,y=y,yaw=yaw),stamp,stamp,i*.1 if now is None else now)
    def motion(self,vx=0.,vy=0.,yaw=0.):
        for i in range(4):row=self.feed(i,vx*i*.1,vy*i*.1,yaw)
        return row
    def test_stationary(self):self.assertEqual(self.motion()['twist'],dict(vx=0.,vy=0.,wz=0.))
    def test_forward(self):self.assertAlmostEqual(self.motion(.2)['twist']['vx'],.2)
    def test_reverse(self):self.assertAlmostEqual(self.motion(-.2)['twist']['vx'],-.2)
    def test_lateral(self):self.assertAlmostEqual(self.motion(0,.1)['twist']['vy'],.1)
    def test_body_frame(self):
        row=self.motion(0,.2,math.pi/2)
        self.assertAlmostEqual(row['twist']['vx'],.2);self.assertAlmostEqual(row['twist']['vy'],0.)
    def test_rotation_and_wrap(self):
        for i in range(4):r=self.feed(i,yaw=math.atan2(math.sin(math.pi-.02+i*.02),math.cos(math.pi-.02+i*.02)))
        self.assertAlmostEqual(r['twist']['wz'],.2)
    def test_warmup(self):
        self.assertFalse(self.feed(0)['valid']);self.assertFalse(self.feed(1)['valid']);self.assertTrue(self.feed(2)['valid'])
    def test_duplicate(self):
        self.motion();r=self.feed(3);self.assertFalse(r['valid']);self.assertNotIn('twist',r)
    def test_reverse_stamp_and_recovery(self):
        self.motion();self.assertFalse(self.feed(1)['valid'])
        self.assertFalse(self.feed(2)['valid']);self.assertFalse(self.feed(3)['valid']);self.assertTrue(self.feed(4)['valid'])
    def test_gap_rewarms(self):
        self.motion();self.assertFalse(self.feed(9)['valid']);self.assertFalse(self.feed(10)['valid']);self.assertTrue(self.feed(11)['valid'])
    def test_wall_gap_despite_fresh_stamp(self):
        self.motion();self.assertFalse(self.feed(4,now=2.)['valid'])
    def test_tiny_interval(self):
        self.feed(0);self.assertFalse(self.feed(.01)['valid'])
    def test_jump(self):
        self.motion();r=self.feed(4,x=3.);self.assertEqual(r['reason'],'pose_jump_or_excessive_speed');self.assertNotIn('twist',r)
    def test_rotation_jump(self):
        self.motion();self.assertFalse(self.feed(4,yaw=1.)['valid'])
    def test_stale_and_future(self):
        for now in (1_300_000_000,900_000_000):
            self.assertFalse(self.e.update(dict(x=0,y=0,yaw=0),1_000_000_000,now,0.)['valid'])
    def test_nan_bool_and_missing(self):
        for pose in (dict(x=float('nan'),y=0,yaw=0),dict(x=True,y=0,yaw=0),{}):
            self.assertFalse(self.e.update(pose,1_000_000_000,1_000_000_000,0.)['valid'])
    def test_timeout_has_no_fake_zero(self):
        self.motion(.2);r=self.e.tick(1_700_000_000,.7)
        self.assertFalse(r['valid']);self.assertNotIn('twist',r);self.assertIsNone(self.e.tick(1_800_000_000,.8))
    def test_sensor_clock_reset(self):
        self.motion();self.assertFalse(self.e.tick(1,.31)['valid'])
    def test_quaternion_and_planarity(self):
        tf=NS(translation=NS(x=1.,y=2.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.))
        self.assertEqual(planar_transform(tf),dict(x=1.,y=2.,yaw=0.))
        tf.rotation.w=0.
        with self.assertRaises(ValueError):planar_transform(tf)
        tf.rotation.w=1.;tf.translation.z=.5
        with self.assertRaises(ValueError):planar_transform(tf)


if __name__=='__main__':unittest.main()
