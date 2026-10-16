from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_scan import SearchScan
from search_arrival import SearchArrival
from tf_motion_gate import TfSearchObserver


class ObserverTests(unittest.TestCase):
    def setUp(self):
        self.pose=dict(x=0,y=0,yaw=0)
        self.scan=SearchScan('test','canned_juice',[self.pose],0)
        self.arrival=SearchArrival(self.scan,'goal')
        self.observer=TfSearchObserver(self.arrival,0)

    def feed(self,i,odom=None,map_pose=None):
        ns=1_000_000_000+i*100_000_000
        return self.observer.sample(odom or self.pose,map_pose or self.pose,ns,ns,i*.1)

    def settle(self):
        self.arrival.result('goal',True,1)
        for i in range(25):self.feed(i)
        self.assertEqual(self.scan.phase,'observe')

    def test_no_goal(self):
        for i in range(25):self.feed(i)
        self.assertEqual(self.scan.phase,'navigate')

    def test_stationary_continues(self):
        self.settle();self.feed(25)
        self.assertEqual(self.scan.phase,'observe')

    def test_moving_aborts(self):
        self.settle();self.feed(25,odom=dict(x=.03,y=0,yaw=0))
        self.assertEqual(self.scan.phase,'incomplete')
        self.assertIsNone(self.scan.position)

    def test_timeout_aborts(self):
        self.settle();self.observer.tick(2.8)
        self.assertEqual(self.scan.phase,'incomplete')

    def test_map_jump_aborts(self):
        self.settle();self.feed(25,map_pose=dict(x=.04,y=0,yaw=0))
        self.assertEqual(self.scan.phase,'incomplete')

    def test_duplicate_aborts(self):
        self.settle();self.feed(24)
        self.assertEqual(self.scan.phase,'incomplete')

    def test_missing_map_aborts(self):
        self.settle()
        self.observer.sample(self.pose,None,3_500_000_000,3_500_000_000,2.5)
        self.assertEqual(self.scan.phase,'incomplete')

    def enable_recovery(self):
        self.observer.recover_transient_tf=True
        self.settle()

    def test_transient_gap_requires_new_settling_and_adapter(self):
        self.enable_recovery()
        old_adapter=self.arrival.adapter
        deadline=self.scan.deadline
        row=self.observer.tick(2.8)
        self.assertEqual(row['observer_reason'],'tf_reacquiring')
        self.assertEqual(self.scan.phase,'navigate')
        self.assertIsNone(self.arrival.adapter)
        self.assertEqual(self.scan.negative,0)
        for i in range(30,40):self.feed(i)
        self.assertEqual(self.scan.phase,'navigate')
        for i in range(40,55):self.feed(i)
        self.assertEqual(self.scan.phase,'observe')
        self.assertIsNot(self.arrival.adapter,old_adapter)
        self.assertEqual(self.scan.deadline,deadline)
        self.assertGreater(self.arrival.adapter.arrival,3_400_000_000)

    def test_recovery_has_fixed_deadline(self):
        self.enable_recovery();self.observer.tick(2.8)
        deadline=self.observer.recovery_deadline
        for now in (3.,4.,5.,6.):self.observer.tick(now)
        self.assertEqual(self.observer.recovery_deadline,deadline)
        self.observer.tick(6.81)
        self.assertEqual(self.scan.phase,'incomplete')
        self.assertEqual(self.scan.failures,['tf_stream_timeout'])
        self.feed(70)
        self.assertEqual(self.scan.phase,'incomplete')

    def test_second_outage_is_terminal(self):
        self.enable_recovery();self.observer.tick(2.8)
        for i in range(30,55):self.feed(i)
        self.assertEqual(self.scan.phase,'observe')
        self.observer.tick(5.8)
        self.assertEqual(self.scan.phase,'incomplete')

    def test_recovery_never_extends_overall_deadline(self):
        self.enable_recovery();self.scan.deadline=3.5
        self.observer.tick(2.8);self.observer.tick(3.5)
        self.assertEqual(self.scan.phase,'incomplete')
        self.assertIn('search_timeout',self.scan.failures)

    def test_cancel_during_recovery_cannot_resume(self):
        self.enable_recovery();self.observer.tick(2.8);self.scan.cancel()
        for i in range(30,60):self.feed(i)
        self.assertEqual(self.scan.phase,'cancelled')

    def test_recovery_rechecks_map_position(self):
        self.enable_recovery();self.observer.tick(2.8)
        for i in range(30,70):self.feed(i,map_pose=dict(x=.5,y=0,yaw=0))
        self.assertEqual(self.scan.phase,'incomplete')
        self.assertIsNone(self.scan.position)

    def test_movement_is_not_recoverable(self):
        self.enable_recovery();self.feed(25,odom=dict(x=.03,y=0,yaw=0))
        self.assertEqual(self.scan.phase,'incomplete')
        self.assertFalse(self.observer.recovery_used)

    def test_resumed_sample_before_timer_still_revalidates(self):
        self.enable_recovery();self.feed(30)
        self.assertEqual(self.scan.phase,'navigate')
        self.assertTrue(self.observer.recovery_used)
        self.assertIsNone(self.arrival.adapter)

    def test_old_frame_rejected_after_recovery(self):
        from rgbd_localization import AUDIT_SHA256
        self.enable_recovery();self.observer.tick(2.8)
        for i in range(30,55):self.feed(i)
        row=dict(target='canned_juice',geometry_verified=True,audit_sha256=AUDIT_SHA256,
                 rgb_stamp_ns=3_400_000_000,depth_stamp_ns=3_400_000_000)
        result=self.arrival.adapter.consume(row,5.45,sensor_now_ns=6_450_000_000)
        self.assertEqual(result['adapter_reason'],'old_or_duplicate_frame')
        self.assertEqual(result['state'],'observe')
        self.assertIsNone(self.scan.position)

if __name__=='__main__':unittest.main()
