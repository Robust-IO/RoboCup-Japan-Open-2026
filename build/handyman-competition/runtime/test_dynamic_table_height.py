import math,unittest
from dynamic_table_height import OFFSET,release_height,lift_targets,extension_retreat
class DynamicHeightTests(unittest.TestCase):
    def test_measured_kitchen_table_is_physically_reachable(self):
        palm=release_height(.746437278382955,.0273729193204409,.02);v=lift_targets(palm)
        self.assertAlmostEqual(v['release_lift_m'],.5166123825348)
        self.assertAlmostEqual(v['extension_lift_m'],.5966123825348)
    def test_different_table_heights_change_both_release_and_preparation(self):
        low=lift_targets(release_height(.5,.03,.02));high=lift_targets(release_height(.75,.03,.02))
        self.assertAlmostEqual(high['release_lift_m']-low['release_lift_m'],.25)
        self.assertAlmostEqual(high['extension_lift_m']-low['extension_lift_m'],.25)
    def test_no_artificial_cleanup_reserve_rejects_physically_reachable_height(self):
        v=lift_targets(OFFSET+.68);self.assertEqual(v['extension_lift_m'],.69)
        self.assertAlmostEqual(v['available_cleanup_raise_m'],.01)
        self.assertAlmostEqual(lift_targets(OFFSET+.69)['available_cleanup_raise_m'],0.)
    def test_physical_limits_and_invalid_measurements_remain(self):
        for v in (OFFSET-.001,OFFSET+.691,float('nan'),float('inf')):
            with self.assertRaises(ValueError):lift_targets(v)
        for v in (-.1,float('nan')):
            with self.assertRaises(ValueError):release_height(v,.03,.02)
    def test_held_offset_and_release_gap_are_used(self):
        self.assertAlmostEqual(release_height(.7,.02,.01),.73)
        self.assertAlmostEqual(release_height(.7,.04,.03),.77)
    def test_retreat_uses_measured_clearance_and_is_one_full_step(self):
        self.assertAlmostEqual(extension_retreat(dict(ready=False,minimum_clearance_m=.03,clearance_m=-.012917)),.062917)
        self.assertEqual(extension_retreat(dict(ready=True,minimum_clearance_m=.03,clearance_m=.04)),0.)
    def test_invalid_extension_measurements_do_not_authorize_motion(self):
        with self.assertRaises(ValueError):extension_retreat(dict(ready=False,minimum_clearance_m=.03,clearance_m=float('nan')))
if __name__=='__main__':unittest.main()
