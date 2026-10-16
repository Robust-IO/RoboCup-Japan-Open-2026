from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_goal_binding import SearchGoalBinding


class BindingTests(unittest.TestCase):
    def setUp(self):
        self.pose=dict(x=1,y=2,yaw=0)
        self.b=SearchGoalBinding('task','layout/room/0',self.pose,'a'*64)
        self.kw=dict(generation=self.b.begin_attempt(),task_id='task',point_id='layout/room/0',
            pose=self.pose,map_sha256='a'*64,goal_id='1'*32,role='final_search_point')

    def test_accept(self):
        r=self.b.accept(**self.kw)
        self.assertTrue(self.b.matches(r['goal_id'],1))
        self.assertFalse(r['actionable'])

    def test_reject_mismatches(self):
        for key,value in [('role','room'),('role','waypoint'),('frame_id','odom'),
                          ('map_sha256','b'*64),('point_id','other'),('task_id','other'),
                          ('pose',dict(x=2,y=2,yaw=0)),('generation',0)]:
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):
                self.b.accept(**dict(self.kw,**{key:value}))

    def test_retry_invalidates_old(self):
        self.b.accept(**self.kw)
        self.b.begin_attempt()
        self.assertFalse(self.b.matches('1'*32,1))
        with self.assertRaises(ValueError):self.b.accept(**self.kw)
        with self.assertRaises(ValueError):self.b.accept(**dict(self.kw,generation=2))
        self.b.accept(**dict(self.kw,generation=2,goal_id='2'*32))

    def test_cancel(self):
        self.b.accept(**self.kw);self.b.cancel()
        self.assertFalse(self.b.matches('1'*32,1))

    def test_duplicate(self):
        self.b.accept(**self.kw)
        with self.assertRaises(ValueError):self.b.accept(**self.kw)

    def test_bad_uuid(self):
        for key in ('0'*32,'invalid'):
            with self.assertRaises(ValueError):self.b.accept(**dict(self.kw,goal_id=key))

if __name__=='__main__':unittest.main()
