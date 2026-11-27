import unittest
from perception_bridge import observation_result,held_geometry,placement_height_plan
I=[[1,0,0],[0,1,0],[0,0,1]]
class BridgeTests(unittest.TestCase):
    def test_room_and_support(self):
        a=dict(target='dog_doll',source_table='white_side_table#living_room',points_map=[[1,1,.6]],
               room_polygon=[[0,0],[2,0],[2,2],[0,2]],source_support_verified=True,evidence_ref='current')
        self.assertTrue(observation_result(**a)['confirmed'])
        a['points_map']=[[3,1,.6]]
        self.assertEqual(observation_result(**a)['reason'],'target_outside_living_room')
        a['points_map']=[[1,1,.6]];a['source_support_verified']=False
        self.assertEqual(observation_result(**a)['status'],'incomplete')
    def test_rotated_held_shape(self):
        a=dict(points_map=[[1.1,2,3]],palm_position_map=[1,2,3],palm_rotation_map=I,
               release_rotation_map=I,hold_verified=True,evidence_ref='current')
        self.assertAlmostEqual(held_geometry(**a)['bottom_below_palm_m'],0)
        a['release_rotation_map']=[[0,0,1],[0,1,0],[-1,0,0]]
        self.assertAlmostEqual(held_geometry(**a)['bottom_below_palm_m'],.1)
        a['hold_verified']=False
        with self.assertRaises(ValueError): held_geometry(**a)
    def test_reflection_rejected(self):
        with self.assertRaises(ValueError):
            held_geometry(points_map=[[0,0,0]],palm_position_map=[0,0,0],palm_rotation_map=[[-1,0,0],[0,1,0],[0,0,1]],release_rotation_map=I,hold_verified=True,evidence_ref='current')
    def test_existing_placement_core(self):
        import importlib.util
        from pathlib import Path
        p=Path(__file__).resolve().parents[2]/'handyman_common_placement_v1/frozen/table_transfer_core.py'
        if not p.exists(): p=Path('/__HANDYMAN_PACKAGE__/tools/apple-transfer-auto-v1/table_transfer_core.py')
        spec=importlib.util.spec_from_file_location('table_transfer_core',p)
        core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)
        g=dict(bottom_below_palm_m=.08)
        r=placement_height_plan(core,g,measured_table_height=.4,current_lift=.2,bottom_observed=True)
        self.assertAlmostEqual(r['palm_height_m'],.495)
        self.assertFalse(r['release_authorized'])
        with self.assertRaises(ValueError): placement_height_plan(core,g,measured_table_height=.4,current_lift=.2,bottom_observed=False)
