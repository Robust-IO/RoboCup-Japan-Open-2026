import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from owned_goal_registry import OwnedGoalRegistry


class RegistryTests(unittest.TestCase):
    def setUp(self):
        self.pose=dict(x=1.,y=2.,yaw=0.)
        self.registry=OwnedGoalRegistry('a'*32,'LayoutA/kitchen/0','b'*64,
            dict(route_waypoint=[self.pose],final_search_point=[self.pose]))
        self.row=dict(schema='handyman-owned-goal-v1',task_id='a'*32,point_id='LayoutA/kitchen/0',
            map_sha256='b'*64,goal_id='c'*32,generation=2,role='route_waypoint',pose=self.pose,
            frame_id='map',stamp_ns=100)

    def test_waypoint_and_retry(self):
        self.assertTrue(self.registry.register(self.row,100))
        self.assertTrue(self.registry.register(dict(self.row,goal_id='d'*32,generation=4,role='final_search_point'),100))
        self.assertEqual(len(self.registry.entries),2)

    def test_late_older_generation_is_still_owned(self):
        self.registry.register(self.row,100)
        self.assertTrue(self.registry.register(dict(self.row,goal_id='d'*32,generation=1),100))

    def test_duplicate_idempotent(self):
        self.registry.register(self.row,100)
        self.assertFalse(self.registry.register(self.row,100))

    def test_conflicting_uuid_and_generation(self):
        self.registry.register(self.row,100)
        for row in (dict(self.row,generation=3),dict(self.row,goal_id='d'*32)):
            with self.assertRaises(ValueError):self.registry.register(row,100)

    def test_wrong_identity_or_pose(self):
        for row in (dict(self.row,task_id='d'*32),dict(self.row,map_sha256='e'*64),
                    dict(self.row,pose=dict(x=3.,y=2.,yaw=0.)),dict(self.row,frame_id='odom'),
                    dict(self.row,role='destination')):
            with self.assertRaises(ValueError):self.registry.register(row,100)

    def test_stale_or_invalid(self):
        with self.assertRaises(ValueError):self.registry.register(self.row,2_000_000_101)
        for row in (dict(self.row,goal_id='0'*32),dict(self.row,generation=True),
                    dict(self.row,pose=dict(x=float('nan'),y=2.,yaw=0.))):
            with self.assertRaises(ValueError):self.registry.register(row,100)


if __name__=='__main__':unittest.main()
