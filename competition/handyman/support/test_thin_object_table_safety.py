import copy,unittest
import numpy as np
from thin_object_table_safety import plan,require_plan

def fixture():
    e=dict(all_physical_colliders_included=True,missing_colliders=[],opening_sweep_bounded=True,instance_verified=True,
        environment_contact_offset_m=.003,uncertainty_bound_m=.002,
        samples=[dict(parts=[dict(path='physical-finger',is_trigger=False,vertices_palm_m=[[0,0,-.018],[0,0,.018]],contact_offset_m=.003)])],
        contact_regions=[dict(vertices_palm_m=[[0,0,-.0175],[0,0,.0175]]) for _ in range(2)])
    return dict(center_m=[.7,0,.51125],height_m=.0225),e

class ThinSafetyTest(unittest.TestCase):
    def test_full_hand_floor_replaces_object_center_alignment(self):
        m,e=fixture();p=plan(m,[0,0,.5],[.63,0],np.eye(3),e)
        self.assertGreater(p['palm_height_m'],.53);self.assertTrue(p['feasible']);self.assertFalse(p['grip_at_object_center_required'])
        with self.assertRaises(ValueError):require_plan(p,.5195)
    def test_no_contact_space_means_change_posture_not_lower_more(self):
        m,e=fixture();e['samples'][0]['parts'][0]['vertices_palm_m'][0][2]=-.06
        p=plan(m,[0,0,.5],[.63,0],np.eye(3),e);self.assertFalse(p['feasible']);self.assertTrue(p['requires_different_posture'])
        with self.assertRaises(ValueError):require_plan(p,p['palm_height_m'])
    def test_missing_hidden_colliders_cannot_pass(self):
        for key in ('all_physical_colliders_included','opening_sweep_bounded','instance_verified'):
            m,e=fixture();e[key]=False
            with self.subTest(key=key),self.assertRaises(ValueError):plan(m,[0,0,.5],[.63,0],np.eye(3),e)
    def test_contact_offsets_and_uncertainty_are_not_ignored(self):
        m,e=fixture();a=plan(m,[0,0,.5],[.63,0],np.eye(3),e);e['environment_contact_offset_m']+=.01
        b=plan(m,[0,0,.5],[.63,0],np.eye(3),e);self.assertAlmostEqual(b['palm_height_m']-a['palm_height_m'],.01)
    def test_closing_sweep_can_have_lower_point_than_open_hand(self):
        m,e=fixture();a=plan(m,[0,0,.5],[.63,0],np.eye(3),e);s=copy.deepcopy(e['samples'][0]);s['parts'][0]['vertices_palm_m'][0][2]-=.006;e['samples'].append(s)
        b=plan(m,[0,0,.5],[.63,0],np.eye(3),e);self.assertAlmostEqual(b['palm_height_m']-a['palm_height_m'],.006)
if __name__=='__main__':unittest.main()
