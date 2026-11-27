import unittest
import numpy as np
from support_reference import make_reference,assess_bottom
class ReferenceTests(unittest.TestCase):
    def context(self):
        points=[[0,0,.41],[.05,0,.46],[0,.05,.45]]
        return dict(vertices_palm_m=(np.array(points)-[0,0,.5]).tolist(),source_support_reference=make_reference(points,[0,0,.4],[0,0,.5],np.eye(3),10))
    def test_supported_orientation_estimates_table_level(self):
        r=assess_bottom(self.context(),np.eye(3));self.assertTrue(r['established'])
        self.assertAlmostEqual(r['bottom_below_palm_m'],.1);self.assertFalse(r['underside_directly_observed'])
    def test_rotation_cannot_reuse_old_bottom(self):
        r=assess_bottom(self.context(),[[0,0,1],[0,1,0],[-1,0,0]])
        self.assertFalse(r['established'])
    def test_missing_reference_not_assumed(self):self.assertFalse(assess_bottom({},np.eye(3))['established'])
    def test_floating_surface_not_support_evidence(self):
        with self.assertRaises(ValueError):make_reference([[0,0,.6]],[0,0,.4],[0,0,.5],np.eye(3),10)
