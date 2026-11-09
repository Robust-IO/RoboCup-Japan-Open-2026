import unittest
from inverse_table_place_plan import build,release_height


class InversePlaceTest(unittest.TestCase):
    def test_prepare_has_no_open_command(self):
        plan=build('prepare',.45,.4149)
        self.assertTrue(plan['requires_outside_table_before_prepare'])
        self.assertTrue(all(s['kind']=='arm' and s['goal'][5]==.4149 for s in plan['steps']))
        self.assertEqual(plan['steps'][2]['goal'][1],-1.7)
        self.assertAlmostEqual(plan['steps'][-1]['goal'][0]+.2771978151685961,.45)
    def test_stages_join(self):
        for before,after in [('prepare','release'),('release','raise'),('raise','retract')]:
            self.assertEqual(build(before,.45,.4149)['steps'][-1]['goal'],build(after,.45,.4149)['initial'])
    def test_height_uses_object_bottom(self):self.assertAlmostEqual(release_height(.4,.025),.44)
    def test_invalid_geometry(self):
        for args in [(0,.02),(.4,-.01),(.8,.02)]:
            with self.assertRaises(ValueError):release_height(*args)
    def test_invalid_holding_or_stage(self):
        for args in [('prepare',.45,0),('close',.45,.41),('prepare',.9,.41)]:
            with self.assertRaises(ValueError):build(*args)


if __name__=='__main__':unittest.main()
