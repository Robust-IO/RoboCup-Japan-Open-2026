from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_arrival_ros import validate_binding


class WireTests(unittest.TestCase):
    def setUp(self):
        self.args=SimpleNamespace(task_id='task',point_id='point',x=1.,y=2.,yaw=0.,map_sha256='a'*64)
        self.row=dict(schema='handyman-search-binding-v1',stamp_ns=1_000_000_000,
            task_id='task',point_id='point',pose=dict(x=1.,y=2.,yaw=0.),map_sha256='a'*64,
            goal_id='1'*32,generation=3,role='final_search_point',frame_id='map')

    def test_valid(self):
        self.assertEqual(validate_binding(self.row,self.args,1_100_000_000)['goal_id'],'1'*32)

    def test_wrong_identity(self):
        for key,value in [('task_id','old'),('point_id','wrong'),('map_sha256','b'*64),
                          ('role','waypoint'),('frame_id','odom'),('generation',0)]:
            with self.subTest(key=key),self.assertRaises(ValueError):
                validate_binding(dict(self.row,**{key:value}),self.args,1_100_000_000)

    def test_stale_or_future(self):
        for now in (999_999_999,3_000_000_001):
            with self.assertRaises(ValueError):validate_binding(self.row,self.args,now)

    def test_bad_schema(self):
        for row in ([],None,{}):
            with self.assertRaises(ValueError):validate_binding(row,self.args,1_100_000_000)

if __name__=='__main__':unittest.main()
