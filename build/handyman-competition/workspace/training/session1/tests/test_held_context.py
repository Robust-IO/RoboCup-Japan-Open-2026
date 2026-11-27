import unittest
import importlib.util
from pathlib import Path
import numpy as np
p=Path(__file__).resolve().parents[1]/'grasp_runtime/autograsp/held_context.py'
spec=importlib.util.spec_from_file_location('held_context',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class HeldContextTests(unittest.TestCase):
    def args(self):
        return dict(points_base=[[1,2,3]],palm_translation_base=[1,2,2],
                    palm_rotation_base=np.eye(3),stamp_ns=123,target='dog_doll',
                    source_table='white_side_table#living_room',joints=[0,0,0,0,0,.3],
                    lift_evidence=dict(verified=True,object_followed=True,grip_retained=True,observations=3))
    def test_current_surface_is_palm_relative(self):
        r=m.build_context(**self.args())
        self.assertEqual(r['vertices_palm_m'],[[0,0,1]])
        self.assertEqual(r['frame'],'hand_palm_link')
        self.assertFalse(r['carry_ready']);self.assertFalse(r['release_authorized'])
    def test_rotated_palm(self):
        a=self.args();a['palm_rotation_base']=[[0,0,1],[0,1,0],[-1,0,0]]
        self.assertEqual(m.build_context(**a)['vertices_palm_m'],[[-1,0,0]])
    def test_unverified_lift_not_exportable(self):
        a=self.args();a['lift_evidence']['object_followed']=False
        with self.assertRaises(ValueError):m.build_context(**a)
    def test_nonfinite_and_invalid_stamp(self):
        a=self.args();a['points_base']=[[float('nan'),0,0]]
        with self.assertRaises(ValueError):m.build_context(**a)
        a=self.args();a['stamp_ns']=0
        with self.assertRaises(ValueError):m.build_context(**a)
