import copy,math,unittest
import numpy as np
from pepper_hold import current_cap_depth,palm_points
from pepper_geometry import estimate_source
from pepper_transfer_automation import validate_lifted_hold
from test_pepper_geometry import scene

class LiftedHoldTests(unittest.TestCase):
    def setUp(self):
        rgb,d,s,det,b,c=scene();model=estimate_source(rgb,d,s,det)
        self.rgb,self.d,self.s,_,b,c=scene(height_delta=.1,remove_body=True)
        self.palm=dict(translation=(b-[.08,0,0]).tolist(),quaternion=[0.,0.,0.,1.])
        bp,cp=palm_points(b,c,self.palm)
        self.held=dict(class_name='ground_pepper',grasp_lift_verified=True,model=model,body_palm_m=bp,cap_palm_m=cp,holding_opening_rad=.4,
            run_id='source',run_root='source-root',boot_id='current')
        self.s['joints']=dict(hand_motor_joint=.4)
    def test_visible_cap_can_continue_verified_hold_without_claiming_hidden_body(self):
        proof=current_cap_depth(self.rgb,self.d,self.s,self.palm,self.held)
        self.assertTrue(proof['current_cap_depth_verified']);self.assertFalse(proof['full_body_measurement_available'])
        self.assertFalse(proof['new_target_identity_created']);self.assertIsNone(proof['measurement'])
        self.assertLess(proof['palm_cap_offset_error_m'],.002)
    def test_missing_cap_zero_depth_or_changed_grip_cannot_continue(self):
        with self.assertRaises(ValueError):current_cap_depth(self.rgb,self.d*0,self.s,self.palm,self.held)
        rgb,d,s,_,_,_=scene(height_delta=.1,remove_cap=True);s['joints']=dict(hand_motor_joint=.4)
        with self.assertRaises(ValueError):current_cap_depth(rgb,d,s,self.palm,self.held)
        self.s['joints']['hand_motor_joint']=.65
        with self.assertRaisesRegex(ValueError,'grip_not_preserved'):current_cap_depth(self.rgb,self.d,self.s,self.palm,self.held)
    def test_cap_is_not_identity_or_grasp_proof_from_scratch(self):
        self.held['grasp_lift_verified']=False
        with self.assertRaisesRegex(ValueError,'verified_ground_pepper_hold'):current_cap_depth(self.rgb,self.d,self.s,self.palm,self.held)
    def test_continuation_requires_previous_verified_lift_and_matching_current_posture(self):
        state=dict(joints=[.294,-1.7,0.,.1292036732051034,0.,.4])
        meta=dict(run_id='source',run_root='source-root',boot_id='current')
        previous=dict(status='stopped_requires_actual_state_review',class_name='ground_pepper',stage='verify_hold_before_retraction',
            grasp_success_verified=True,close_commands_sent=1,release_commands_sent=0,confirmed_stages=['grasp_lift'],last_fresh_robot_state=state)
        validate_lifted_hold(previous,meta,self.held,state,'current')
        for change in (dict(release_commands_sent=1),dict(grasp_success_verified=False),dict(stage='verify_lift')):
            with self.assertRaises(ValueError):validate_lifted_hold(dict(previous,**change),meta,self.held,state,'current')
        with self.assertRaises(ValueError):validate_lifted_hold(previous,meta,self.held,state,'another-boot')

if __name__=='__main__':unittest.main()
