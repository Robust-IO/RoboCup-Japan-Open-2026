import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_scan import SearchScan
from search_arrival import SearchArrival


class ArrivalTests(unittest.TestCase):
    def setUp(self):
        self.scan=SearchScan('task','canned_juice',[dict(x=1,y=2,yaw=0)],0)
        self.gate=SearchArrival(self.scan,'goal-1')

    def sample(self,i,**kwargs):
        args=dict(pose=dict(x=1,y=2,yaw=0),linear_speed=0,angular_speed=0,
                  stamp_ns=1_000_000_000+i*250_000_000,
                  sensor_now_ns=1_000_000_000+i*250_000_000,now=i*.25)
        args.update(kwargs)
        return self.gate.sample(**args)

    def success(self):
        self.assertTrue(self.gate.result('goal-1',True,1_000_000_000))

    def test_requires_matching_success(self):
        self.assertFalse(self.gate.result('old-goal',True,1))
        self.assertEqual(self.sample(1)['arrival_reason'],'waiting_for_matching_goal_success')

    def test_one_second_then_vision_adapter(self):
        self.success()
        for i in range(1,5): self.assertEqual(self.sample(i)['state'],'navigate')
        self.assertEqual(self.sample(5)['state'],'observe')
        self.assertEqual(self.gate.adapter.arrival,2_250_000_000)

    def test_moving_resets(self):
        self.success();self.sample(1);self.sample(2)
        self.assertEqual(self.sample(3,linear_speed=.1)['arrival_reason'],'robot_still_moving')
        self.sample(4)
        self.assertEqual(self.sample(5)['state'],'navigate')

    def test_wrong_pose_and_yaw(self):
        self.success()
        for i,pose in enumerate([dict(x=2,y=2,yaw=0),dict(x=1,y=2,yaw=1)],1):
            self.assertEqual(self.sample(i,pose=pose)['arrival_reason'],'outside_search_pose_tolerance')

    def test_duplicate_and_stale(self):
        self.success();self.sample(1)
        self.assertEqual(self.sample(1)['arrival_reason'],'stale_or_duplicate_pose')
        self.assertEqual(self.sample(2,sensor_now_ns=9_000_000_000)['arrival_reason'],'stale_or_duplicate_pose')

    def test_gap_restarts(self):
        self.success();self.sample(1);self.sample(2)
        self.assertEqual(self.sample(8)['arrival_reason'],'settling')

    def test_failure_not_absence(self):
        self.gate.result('goal-1',False,1)
        result=self.sample(1)
        self.assertEqual(result['state'],'navigate')
        self.assertFalse(result['does_not_exist_authorized'])

    def test_cancel_blocks(self):
        self.success();self.scan.cancel()
        self.assertEqual(self.sample(1)['state'],'cancelled')
        self.assertIsNone(self.gate.adapter)

    def test_wrong_frame(self):
        self.success()
        self.assertEqual(self.sample(1,frame_id='odom')['arrival_reason'],'invalid_pose_or_speed')

    def test_pose_drift_despite_zero_speed(self):
        self.success();self.sample(1);self.sample(2)
        self.sample(3,pose=dict(x=1.03,y=2,yaw=0))
        self.assertEqual(self.sample(4)['state'],'navigate')

if __name__=='__main__': unittest.main()
