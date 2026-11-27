import math
import unittest
from reviewed_carry_adjustment import endpoint,single_placement_limit,adjustment_settled


class EndpointTest(unittest.TestCase):
    def test_single_placement_stable_actual_pose_not_exact_command_endpoint(self):
        self.assertTrue(adjustment_settled(.05,.04,.001,single_placement=True))
        self.assertFalse(adjustment_settled(.05,.04,.001))
        self.assertFalse(adjustment_settled(.001,.001,.03,single_placement=True))
        self.assertFalse(adjustment_settled(float('nan'),.01,0,single_placement=True))
    def test_base_frame_translation(self):
        target=endpoint([2,3,math.pi/2],[.2,-.1,0])
        self.assertAlmostEqual(target[0],2.1)
        self.assertAlmostEqual(target[1],3.2)

    def test_rotation_is_finite(self):
        self.assertEqual(endpoint([1,2,0],[0,0,.1]),[1,2,.1])

    def test_reject_unbounded_or_nonfinite(self):
        for d in ([.41,0,0],[0,0,.36],[float('nan'),0,0]):
            with self.assertRaises(ValueError):endpoint([0,0,0],d)

    def test_explicit_single_placement_accepts_full_47cm_without_splitting(self):
        delta=[.473,-.028,0.]
        limit=single_placement_limit(delta,[.2168,-1.7,0,.1292036732051034,0],.40096)
        self.assertEqual(endpoint([0,0,0],delta,max_translation_m=limit),delta)

    def test_explicit_single_placement_has_no_metres_limit(self):
        arm=[.2168,-1.7,0,.1292036732051034,0]
        for length in (.601,1.2,8.):
            delta=[length,-.028,0.]
            limit=single_placement_limit(delta,arm,.4)
            self.assertIsNone(limit)
            self.assertEqual(endpoint([0,0,0],delta,max_translation_m=limit),delta)

    def test_explicit_single_placement_not_backwards_rotating_or_empty(self):
        arm=[.2168,-1.7,0,.1292036732051034,0]
        for delta,pose,opening in [([-.47,0,0],arm,.4),([.47,0,.1],arm,.4),
                                   ([.47,0,0],arm,.65),([.47,0,0],[0]*5,.4)]:
            with self.assertRaises(ValueError):single_placement_limit(delta,pose,opening)

    def test_explicit_placement_does_not_override_identity_based_empty_approach(self):
        with self.assertRaises(ValueError):
            single_placement_limit([.47,0,0],[.2168,-1.7,0,.1292036732051034,0],.4,'old-recording')


if __name__=='__main__':unittest.main()
