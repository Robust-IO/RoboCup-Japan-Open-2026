import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_vision_adapter import SearchVisionAdapter
from search_scan import SearchScan
from rgbd_localization import AUDIT_SHA256

class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.scan=SearchScan('t','canned_juice',[dict(x=0,y=0,yaw=0)],0)
        self.scan.arrived('t:0',True,True,0)
        self.a=SearchVisionAdapter(self.scan,'t:0',1)
    def row(self,i):
        stamp=i*100000000
        return dict(target='canned_juice',geometry_verified=True,audit_sha256=AUDIT_SHA256,
            rgb_stamp_ns=stamp,depth_stamp_ns=stamp,tf_wait_status='ready',tf_at_depth_stamp={},
            quality=dict(stable=True,position_m=[1,2,3],frame_id='odom',target='canned_juice',stamp_ns=stamp),
            inference=dict(detections=[dict(name='canned_juice')],class_conflicts=[]))
    def feed(self,r): return self.a.consume(r,1,sensor_now_ns=500000000)
    def test_three_new_frames(self):
        self.assertEqual(self.feed(self.row(1))['state'],'observe')
        self.feed(self.row(2));self.assertEqual(self.feed(self.row(3))['state'],'found')
    def test_duplicates(self):
        for _ in range(4): self.feed(self.row(1))
        self.assertEqual(self.scan.phase,'observe')
    def test_absent_is_not_coverage(self):
        r=self.row(1);r['inference']['detections']=[]
        self.assertEqual(self.feed(r)['adapter_reason'],'no_target_but_view_coverage_unverified')
        self.assertEqual(self.scan.negative,0)
    def test_outage_resets_streak(self):
        self.feed(self.row(1)); self.feed(dict(target='canned_juice'))
        self.assertEqual(self.feed(self.row(2))['fresh_view_count'],1)
    def test_stale(self):
        self.assertEqual(self.a.consume(self.row(1),1,sensor_now_ns=4000000000)['adapter_reason'],'unsynchronised_or_stale')
    def test_healthy_absence_is_not_coverage(self):
        for i in range(1,4):
            r=self.row(i);r['inference']['detections']=[]
            r['inference']['view_health']=dict(schema='handyman-view-health-v1',data_usable=True,coverage_verified=False)
            self.assertEqual(self.feed(r)['adapter_reason'],'no_target_data_usable_but_coverage_unverified')
        self.assertEqual(self.scan.phase,'observe')
        self.assertEqual(self.scan.negative,0)
    def test_unhealthy_absence(self):
        r=self.row(1);r['inference']['detections']=[]
        r['inference']['view_health']=dict(schema='handyman-view-health-v1',data_usable=False)
        self.assertEqual(self.feed(r)['adapter_reason'],'no_target_with_unusable_view_data')
        self.assertEqual(self.scan.negative,0)
    def test_unhealthy_target_resets_streak(self):
        self.feed(self.row(1))
        r=self.row(2)
        r['inference']['view_health']=dict(schema='handyman-view-health-v1',data_usable=False)
        self.assertEqual(self.feed(r)['adapter_reason'],'target_with_unusable_view_data')
        self.assertEqual(self.feed(self.row(3))['fresh_view_count'],1)
        self.assertEqual(self.scan.phase,'observe')
    def test_healthy_target(self):
        for i in range(1,4):
            r=self.row(i)
            r['inference']['view_health']=dict(schema='handyman-view-health-v1',data_usable=True,coverage_verified=False)
            result=self.feed(r)
        self.assertEqual(result['state'],'found')
        self.assertFalse(result['actionable'])
    def test_unknown_health_rejected(self):
        r=self.row(1);r['inference']['view_health']=dict(schema='future-version',data_usable=True)
        self.assertEqual(self.feed(r)['adapter_reason'],'target_with_unusable_view_data')

if __name__=='__main__': unittest.main()
