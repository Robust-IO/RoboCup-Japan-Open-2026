import math
import unittest
from finite_base_step_plan import plan


class Tests(unittest.TestCase):
    def test_bound(self):self.assertLess(plan((0,0,0))['stationary_source_based_distance_bound_m'],.02)
    def test_frame(self):
        p=plan((1,2,math.pi/2),(1,0));self.assertAlmostEqual(p['positions'][0],1);self.assertAlmostEqual(p['positions'][1],2.015)
    def test_no_authorization(self):self.assertFalse(plan((0,0,0))['motion_authorized'])
    def test_long_step(self):
        with self.assertRaises(ValueError):plan((0,0,0),distance=.02)
    def test_zero_direction(self):
        with self.assertRaises(ValueError):plan((0,0,0),(0,0))
    def test_wrap(self):
        with self.assertRaises(ValueError):plan((0,0,math.pi))
    def test_invalid(self):
        with self.assertRaises(ValueError):plan((float('nan'),0,0))
    def test_short_duration(self):
        with self.assertRaises(ValueError):plan((0,0,0),duration=.1)


if __name__=='__main__':unittest.main()
