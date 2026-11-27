import copy
import unittest
from apple_approach_lease import validate,validate_closure


class Tests(unittest.TestCase):
    def setUp(self):
        self.snapshot=dict(tracking_epoch='epoch',streams={'rgb':{'stamp_ns':1_000_000_000},'depth':{'stamp_ns':1_000_000_000}},
            tracking_transforms={'odom_from_base_at_depth':dict(stamp_ns=1_000_000_000,source_frame='base_link',target_frame='odom',translation=[0,0,0],quaternion=[0,0,0,1])})
        measurement=dict(stamp_ns=1_000_000_000,measurement='fresh_rgbd',class_name='apple',depth_valid=True,
            support_consistent=True,clipped=False,point_odom_m=[.97,.078,.536],radius_m=.032,fit_residual_p95_m=.005,confidence=.31)
        self.observation=dict(tracking_epoch='epoch',identity_tracking_task_id='task',timing={'freshness_passed':True},
            identity_tracking=dict(status='identity_tracked',identity_confirmed=True,measurement=measurement,
                room_evidence=[dict(room_confirmed=True,target_room='living_room')]))
        self.kw=dict(now_ns=1_200_000_000,epoch='epoch',task_id='task',current_odom=[0,0,0],translation=[.1,0],palm_height=.53)

    def check(self):return validate(self.observation,self.snapshot,**self.kw)
    def test_fresh_measured_low_score_identity(self):
        self.assertGreater(self.check()['remaining_target_minus_contact_xy_m'][0],.05)
        self.assertFalse(self.check()['motion_authorized'])
    def test_old_data(self):
        self.kw['now_ns']=1_500_000_000
        with self.assertRaises(ValueError):self.check()
    def test_task_switch(self):
        self.kw['task_id']='other'
        with self.assertRaises(ValueError):self.check()
    def test_identity_reset(self):
        self.observation['identity_tracking']['status']='reobserve'
        with self.assertRaises(ValueError):self.check()
    def test_prediction_rejected(self):
        self.observation['identity_tracking']['measurement']['measurement']='predicted'
        with self.assertRaises(ValueError):self.check()
    def test_base_shift(self):
        self.kw['current_odom']=[.02,0,0]
        with self.assertRaises(ValueError):self.check()
    def test_step_overshoots(self):
        self.observation['identity_tracking']['measurement']['point_odom_m'][0]=.82
        with self.assertRaises(ValueError):self.check()
    def test_height_mismatch(self):
        self.kw['palm_height']=.6
        with self.assertRaises(ValueError):self.check()
    def test_wrong_room(self):
        self.observation['identity_tracking']['room_evidence'][0]['target_room']='lobby'
        with self.assertRaises(ValueError):self.check()
    def test_explicit_large_close_approach_policy(self):
        self.observation['identity_tracking']['measurement']['point_odom_m'][0]=.874
        self.kw.update(translation=[.17,0],max_step_m=.25,min_forward_gap_m=.01)
        self.assertGreater(self.check()['remaining_target_minus_contact_xy_m'][0],.01)
        self.kw['translation']=[.18,0]
        with self.assertRaises(ValueError):self.check()
    def closure(self,point):
        self.observation['identity_tracking']['measurement']['point_odom_m']=point
        args={k:v for k,v in self.kw.items() if k!='translation'}
        return validate_closure(self.observation,self.snapshot,**args)
    def test_current_closure_alignment(self):
        r=self.closure([.7047,.0742,.5397])
        self.assertTrue(r['trial_only']);self.assertFalse(r['grasp_center_verified'])
    def test_closure_too_far(self):
        with self.assertRaises(ValueError):self.closure([.74,.078,.536])
    def test_closure_lateral_misalignment(self):
        with self.assertRaises(ValueError):self.closure([.70,.10,.536])
    def test_closure_old_identity(self):
        self.kw['now_ns']+=1_000_000_000
        with self.assertRaises(ValueError):self.closure([.70,.078,.536])


if __name__=='__main__':unittest.main()
