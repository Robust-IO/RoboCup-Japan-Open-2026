import unittest
from can_approach_plan import approach_plan


class CanApproachPlanTests(unittest.TestCase):
    def test_actual_failed_diagonal_plan_aligns_sideways_first(self):
        p=approach_plan([.3812773912139785,-.1339412336750701,0.])
        self.assertEqual(p['phase'],'lateral_alignment');self.assertEqual(p['delta'],[0.,-.1339412336750701,0.])
    def test_aligned_can_uses_entire_forward_distance_once(self):
        for distance in (.21,.3812773912139785,.75,1.2):
            p=approach_plan([distance,0.,0.])
            self.assertEqual(p['phase'],'straight_contact_approach');self.assertEqual(p['delta'],[distance,0.,0.])
    def test_nearby_lateral_correction_first_withdraws_fingers(self):
        p=approach_plan([.01,.03,0.]);self.assertEqual(p['phase'],'clear_before_lateral_alignment')
        self.assertAlmostEqual(p['delta'][0],-.14);self.assertEqual(p['delta'][1],0.)
        q=approach_plan([.15,.03,0.]);self.assertEqual(q['phase'],'lateral_alignment')
        r=approach_plan([.15,0.,0.]);self.assertEqual(r['delta'],[.15,0.,0.])
    def test_small_odometry_error_does_not_create_millimetre_retreat_loop(self):
        for distance in (.131,.145,.149,.151):
            self.assertEqual(approach_plan([distance,.03,0.])['phase'],'lateral_alignment')
    def test_permitted_lateral_error_does_not_create_diagonal_contact(self):
        for side in (-.004,0.,.004):
            self.assertEqual(approach_plan([.30,side,0.])['delta'],[.30,0.,0.])
    def test_old_accepted_lateral_errors_are_corrected_before_contact(self):
        for side in (-.012,-.005,.005,.012):
            self.assertEqual(approach_plan([.30,side,0.])['phase'],'lateral_alignment')
            self.assertEqual(approach_plan([.01,side,0.])['phase'],'clear_before_lateral_alignment')
    def test_invalid_height_and_nonfinite_error_do_not_produce_a_command(self):
        for error in ([.3,0.,.021],[float('nan'),0.,0.],[0.,float('inf'),0.],[0.,0.]):
            with self.assertRaises(ValueError):approach_plan(error)


if __name__=='__main__':unittest.main()
