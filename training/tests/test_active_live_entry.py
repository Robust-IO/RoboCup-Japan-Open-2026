import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from active_live_entry import validate_live_ring,check_plan_result,plan_current_path,PREFIX


class LiveEntryTests(unittest.TestCase):
    def setUp(self):
        self.args=SimpleNamespace(allow_live=True,multi_point=True,active_bundle='manifest',point_index=0,
            head_tilt=-.25,session_budget_seconds=180,view_seconds=2,action_name='/navigate_to_pose',
            map_topic='/map',vision_topic='/handyman/vision/diagnostics',request_topic=PREFIX+'/request',
            status_topic=PREFIX+'/status',result_topic=PREFIX+'/result',private_prefix=PREFIX+'/runtime')
        self.env={'ROS_DOMAIN_ID':'71','ROS_LOCALHOST_ONLY':'1'}
        self.bundle=dict(task_id='task',map_bundle_sha256='digest',main=dict(x=1,y=2,yaw=0))
        self.report=dict(success=True,action_status=4,task_id='task',map_bundle_sha256='digest',
                         target=self.bundle['main'],start_source='current_robot_tf',robot_control_started=False)

    def test_explicit_gate(self):validate_live_ring(self.args,self.env)
    def test_missing_requirements(self):
        for key,value in [('active_bundle',None),('multi_point',False),('head_tilt',None),('session_budget_seconds',None),('point_index',1)]:
            args=copy.copy(self.args);setattr(args,key,value)
            with self.subTest(key=key),self.assertRaises(ValueError):validate_live_ring(args,self.env)
    def test_wrong_domain(self):
        with self.assertRaises(ValueError):validate_live_ring(self.args,{'ROS_DOMAIN_ID':'73','ROS_LOCALHOST_ONLY':'1'})
    def test_competition_topic_rejected(self):
        self.args.request_topic='/handyman/search/request'
        with self.assertRaises(ValueError):validate_live_ring(self.args,self.env)
    def test_plan_identity(self):check_plan_result(self.report,self.bundle)
    def test_plan_rejected(self):
        for key,value in [('success',False),('task_id','other'),('map_bundle_sha256','other'),('target',{}),('start_source','saved_pose')]:
            report=dict(self.report);report[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):check_plan_result(report,self.bundle)
    def test_planning_failure_propagates(self):
        binding=SimpleNamespace(check=lambda:None,path=Path('/unused'),bundle=self.bundle)
        with patch('active_live_entry.subprocess.run',side_effect=TimeoutError('planner timeout')):
            with self.assertRaises(TimeoutError):plan_current_path(binding,Path('/unused-output'))
    def test_bundle_failure_prevents_planner(self):
        def fail():raise ValueError('changed bundle')
        binding=SimpleNamespace(check=fail)
        with patch('active_live_entry.subprocess.run') as call:
            with self.assertRaises(ValueError):plan_current_path(binding,Path('/unused-output'))
            call.assert_not_called()


if __name__=='__main__':unittest.main()
