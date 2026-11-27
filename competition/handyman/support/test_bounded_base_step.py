import math
import unittest
from bounded_base_step import BaseStep


class Tests(unittest.TestCase):
    def setUp(self):
        self.controller=BaseStep((0,0,0),(.02,0),0)
        self.flags=dict(tf_age=.05,scan_age=.05,joints_age=.05,collision_clear=True,exclusive=True,tucked=True)

    def tick(self, pose=(0,0,0), now=.05, **kwargs):
        return self.controller.update(pose,now,**dict(self.flags,**kwargs))

    def test_speed_bound(self):self.assertEqual(self.tick()['vx'],.01)
    def test_step_limit(self):
        with self.assertRaises(ValueError):BaseStep((0,0,0),(.02,.02),0)
    def test_stale_is_terminal(self):
        self.assertEqual(self.tick(tf_age=.401)['state'],'stale_feedback')
        self.assertEqual(self.tick(now=.1)['vx'],0)
    def test_conflict(self):self.assertEqual(self.tick(exclusive=False)['state'],'control_conflict')
    def test_unknown_space_stops(self):self.assertEqual(self.tick(collision_clear=False)['vx'],0)
    def test_tucked_required(self):self.assertEqual(self.tick(tucked=False)['state'],'arm_not_tucked')
    def test_yaw_drift(self):self.assertEqual(self.tick(pose=(0,0,.04))['state'],'yaw_drift')
    def test_cross_track(self):self.assertEqual(self.tick(pose=(0,.006,0))['state'],'path_drift')
    def test_loop_gap(self):self.assertEqual(self.tick(now=.3)['state'],'control_loop_gap')
    def test_nan(self):self.assertEqual(self.tick(scan_age=float('nan'))['vx'],0)
    def test_completion_latches(self):
        self.tick(pose=(.009,0,0));self.tick(pose=(.018,0,0),now=.1)
        self.assertEqual(self.tick(now=.15)['state'],'step_complete')
    def test_rotated_frame(self):
        self.controller=BaseStep((0,0,math.pi/2),(.02,0),0)
        self.assertAlmostEqual(self.tick(pose=(0,.005,math.pi/2))['vx'],.01)


if __name__=='__main__':unittest.main()
