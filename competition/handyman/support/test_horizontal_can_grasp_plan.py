import math
import unittest
import numpy as np
from horizontal_can_grasp_plan import forward,overhead_plan,validate_goal,CONTACT_LOCAL

class HorizontalPlanTest(unittest.TestCase):
    def test_fk_matches_measured_tucked_palm(self):
        p,r=forward([0,0,0,-math.pi/2,0,0])
        np.testing.assert_allclose(p,[.2866020328,.078,.6764923576],atol=2e-7)
        np.testing.assert_allclose(r@CONTACT_LOCAL,[.0794860814,-.00005,0],atol=1e-8)

    def test_horizontal_pose_matches_existing_arm_height(self):
        p,r=forward([.25,-1.7,0,.1292036732,0,.65])
        self.assertAlmostEqual(p[2],.5271978152,places=6)

    def test_overhead_fingers_close_across_short_dimension(self):
        for yaw in (-1.5,-.4,0,.6,1.62,2.8):
            a=np.array([math.cos(yaw),math.sin(yaw),0])
            m=dict(center_base_m=[.87,.026,.534],axis_base=a.tolist(),radius_m=.0377)
            plan=overhead_plan(m);p,r=forward(plan['goal'])
            self.assertLess(abs(np.dot(r[:,1],a)),1e-9)
            np.testing.assert_allclose(r[:,2],[0,0,-1],atol=1e-8)
            self.assertAlmostEqual((p+r@CONTACT_LOCAL)[2],.684,places=7)
            self.assertTrue(0<=plan['grasp_lift_m']<=.69)

    def test_invalid_goals_do_not_clamp_silently(self):
        for j in ([.70,-1.57,0,-1.57,0,.65],[.4,-1.57,0,-1.57,4.,.65],[.4,-1.57,.1,-1.57,0,.65]):
            with self.assertRaises(ValueError):validate_goal(j)

if __name__=='__main__':unittest.main()
