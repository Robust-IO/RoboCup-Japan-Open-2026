import unittest
from execution_plan import (SOURCE, furniture_candidates, transfer_config,
                            navigation_argv, grasp_argv, grasp_result)

class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.env = {'destinations': {
            'white_side_table': [dict(room='bedroom', x=99,y=99,yaw=0),
                                 dict(room='living_room',x=1,y=2,yaw=3)],
            'square_low_table': [dict(room='living_room',x=4,y=5,yaw=6)]}}

    def test_same_furniture_other_room_excluded(self):
        self.assertEqual(furniture_candidates(self.env,SOURCE), [[1,2,3]])
        with self.assertRaises(ValueError):
            furniture_candidates(self.env,'white_side_table#kitchen')

    def test_shared_config_not_mutated_and_navigation_settings_preserved(self):
        base = dict(task={'object':'apple'},source_table_id='white_side_table#bedroom_1',
                    navigation_tolerances=[.06,.08,.06],motion_profile='balanced_v1')
        cfg = transfer_config(base,self.env,'/test/map.yaml','sha')
        self.assertEqual(base['task']['object'],'apple')
        self.assertEqual(cfg['task']['object'],'dog_doll')
        self.assertEqual(cfg['source_staging_goal'],[1,2,3])
        self.assertEqual(cfg['navigation_tolerances'],base['navigation_tolerances'])
        self.assertEqual(cfg['motion_profile'],base['motion_profile'])

    def test_irregular_template_and_explicit_execution(self):
        argv = grasp_argv('/tmp/config.json','/tmp/output')
        self.assertIn('templates',argv)
        self.assertIn(SOURCE,argv)
        self.assertNotIn('--execute',argv)
        self.assertIn('--execute',grasp_argv('config','output',execute=True))

    def test_nav_uses_current_opening_and_supplied_route(self):
        with self.assertRaises(ValueError):
            navigation_argv([1,2,3],'out',holding_opening=None)
        argv = navigation_argv([1,2,3],'out',holding_opening=.31,waypoints=[[4,5,6]])
        self.assertEqual(argv[argv.index('--holding-opening')+1],'0.31')
        self.assertEqual(argv[-4:],['--waypoint','4','5','6'])

    def test_plan_and_incomplete_lift_never_count_as_grasp(self):
        self.assertEqual(grasp_result({'status':'candidate_selected'})['status'],'incomplete')
        evidence = dict(verified=True, object_followed=True, grip_retained=True,observations=3)
        self.assertEqual(grasp_result(dict(status='grasp_verified',lift_evidence=evidence))['status'],'ok')
        evidence['grip_retained']=False
        self.assertEqual(grasp_result(dict(status='grasp_verified',lift_evidence=evidence))['status'],'incomplete')

if __name__ == '__main__':
    unittest.main()
