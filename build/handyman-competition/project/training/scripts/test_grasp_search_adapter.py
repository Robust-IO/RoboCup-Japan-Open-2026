import unittest
from grasp_search_adapter import adapt, AUDIT_SHA256


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.row=dict(geometry_verified=True,audit_sha256=AUDIT_SHA256,target='apple',rgb_stamp_ns=10_000_000_000,
            depth_stamp_ns=10_000_000_000,tf_wait_status='ready',quality=dict(target='apple',stable=True,frame_id='odom',stamp_ns=10_000_000_000,position_m=[1.,2.,.7]),
            inference=dict(detections=[dict(name='apple',confidence=.9,core_pixels=100,valid_pixels=95,depth_p05_median_p95=[.9,.91,.92])]))
        self.kw=dict(task_id='a',expected_task_id='a',target='apple',now_ns=10_100_000_000,
            room=dict(state='inside',stamp_ns=10_000_000_000),base_from_odom=dict(source_frame='odom',target_frame='base_link',stamp_ns=10_000_000_000,translation=[-1,-2,0],quaternion=[0,0,0,1]))
    def test_transform(self):
        r=adapt(self.row,**self.kw)
        self.assertTrue(r['eligible_for_planning']); self.assertEqual(r['evidence'].point,(0.,0.,.7));self.assertFalse(r['motion_authorized'])
    def test_old_tf(self):
        self.kw['base_from_odom']['stamp_ns']-=1
        self.assertFalse(adapt(self.row,**self.kw)['eligible_for_planning'])
    def test_room(self):
        self.kw['room']['state']='outside_target_room'
        self.assertFalse(adapt(self.row,**self.kw)['eligible_for_planning'])
    def test_no_depth(self):
        del self.row['inference']['detections'][0]['depth_p05_median_p95']
        self.assertFalse(adapt(self.row,**self.kw)['eligible_for_planning'])
    def test_duplicate(self):
        self.row['inference']['detections']*=2
        self.assertFalse(adapt(self.row,**self.kw)['eligible_for_planning'])
    def test_stale(self):
        self.kw['now_ns']+=1_000_000_000
        self.assertFalse(adapt(self.row,**self.kw)['eligible_for_planning'])


if __name__=='__main__': unittest.main()
