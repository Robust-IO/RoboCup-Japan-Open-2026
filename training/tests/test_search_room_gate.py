import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_room_gate import RoomGate
from search_vision_adapter import SearchVisionAdapter
from search_scan import SearchScan
from rgbd_localization import AUDIT_SHA256


class RoomTests(unittest.TestCase):
    def setUp(self):
        self.rooms={'living_room':[[0,0],[2,0],[2,3],[0,3]],
                    'lobby':[[2,0],[4,0],[4,3],[2,3]]}
        self.g=RoomGate(self.rooms,'living_room')

    def classify(self,p):return self.g.classify(p,[0,0,0],[0,0,0,1])
    def test_inside(self):self.assertEqual(self.classify([1,1,1])['state'],'inside')
    def test_lobby(self):self.assertEqual(self.classify([3,1,1])['state'],'outside_target_room')
    def test_boundary(self):
        for x in [1.9,2,2.1]:self.assertEqual(self.classify([x,1,1])['state'],'boundary_uncertain')
    def test_unassigned(self):self.assertEqual(self.classify([5,1,1])['state'],'boundary_uncertain')
    def test_overlap(self):
        self.rooms['overlap']=self.rooms['living_room']
        self.assertEqual(RoomGate(self.rooms,'living_room').classify([1,1,1],[0,0,0],[0,0,0,1])['state'],'boundary_uncertain')
    def test_transform_before_room_test(self):
        r=self.g.classify([1,1,1],[2,0,0],[0,0,0,1])
        self.assertEqual(r['state'],'outside_target_room')
        r=self.g.classify([1,-1,1],[0,0,0],[0,0,math.sqrt(.5),math.sqrt(.5)])
        self.assertEqual(r['state'],'inside')
    def test_invalid_transform(self):
        with self.assertRaises(ValueError):self.g.classify([1,1,1],[0,0,0],[0,0,0,0])

    def adapter(self,point,check=None):
        scan=SearchScan('t','canned_juice',[dict(x=1,y=1,yaw=0)],0)
        scan.arrived('t:0',True,True,0)
        scan.room_check=check or (lambda p,stamp:self.classify(p))
        adapter=SearchVisionAdapter(scan,'t:0',1)
        for i in range(1,4):
            stamp=i*100000000
            row=dict(target='canned_juice',geometry_verified=True,audit_sha256=AUDIT_SHA256,
                rgb_stamp_ns=stamp,depth_stamp_ns=stamp,tf_wait_status='ready',tf_at_depth_stamp={},
                quality=dict(stable=True,position_m=point,frame_id='odom',target='canned_juice',stamp_ns=stamp),
                inference=dict(detections=[dict(name='canned_juice')],class_conflicts=[]))
            result=adapter.consume(row,1,sensor_now_ns=500000000)
        return result
    def test_robot_in_living_target_in_lobby_never_found(self):
        r=self.adapter([3,1,1]);self.assertEqual(r['state'],'observe')
        self.assertEqual(r['room_membership']['state'],'outside_target_room')
        self.assertFalse(r['does_not_exist_authorized'])
    def test_inside_target_can_be_found(self):self.assertEqual(self.adapter([1,1,1])['state'],'found')
    def test_missing_tf_never_found(self):
        r=self.adapter([1,1,1],lambda p,s:dict(state='transform_unavailable'))
        self.assertEqual(r['state'],'observe')
    def test_boundary_never_found(self):self.assertEqual(self.adapter([2,1,1])['state'],'observe')
