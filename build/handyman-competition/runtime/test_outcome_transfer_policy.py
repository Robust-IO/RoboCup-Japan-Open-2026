import copy,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import cv2,numpy as np
from outcome_transfer_policy import load,holding_feedback,hold_assessment,HoldEvidence,ClosureEvidence,alignment_tolerances,joint_endpoint_reached,stationary_action_feedback
from outcome_finite_adjustment import endpoint,adjustment_settled
from flat_box_geometry import select_source
from flat_box_place_geometry import verify_placed
from flat_box_partial_top import observe_known
from tabletop_support import support_from_plan
from test_flat_box_place_geometry import fixture

class OutcomePolicyTest(unittest.TestCase):
    def feedback_scene(self,age=.55):
        names=('arm_lift_joint','arm_flex_joint','arm_roll_joint','wrist_flex_joint','wrist_roll_joint','hand_motor_joint')
        s=dict(tracking_epoch='current',streams={n:dict(stamp_ns=1_000_000_000) for n in ('rgb','depth','hand')},
            joints=dict(zip(names,[.25,-1.7,0,.129,0,.65]),head_pan_joint=0.,head_tilt_joint=-.5),tracking_transforms=dict(odom_from_base_at_depth=dict(translation=[0,0,0],quaternion=[0,0,0,1])),
            stop_observe_act=dict(policy='stop_observe_act_v1',generation=0,captured_after_stop=True,stop_verified_stamp_ns=900_000_000))
        now=int(1_000_000_000+age*1e9);state=dict(joints=[.25,-1.7,0,.129,0,.65],head=[0.,-.5],stamp_ns=now-100_000_000,poses=dict(odom=[0,0,0]))
        return s,state,now
    def test_stationary_computation_can_finish_without_retimestamping_the_image(self):
        s,state,now=self.feedback_scene();before=copy.deepcopy(s)
        self.assertFalse(stationary_action_feedback(s,state,now,'current',current_generation=0)['computation_age_is_rejection_gate'])
        self.assertEqual(s,before)
    def test_extended_processing_budget_cannot_hide_movement_or_stale_state(self):
        for mutation in ('base','arm','joints','epoch','sync','head'):
            s,state,now=self.feedback_scene()
            if mutation=='base':state['poses']['odom'][0]=.03
            if mutation=='arm':state['joints'][0]+=.01
            if mutation=='joints':state['stamp_ns']=now-700_000_000
            if mutation=='head':state['head'][0]=.05
            if mutation=='epoch':s['tracking_epoch']='old'
            if mutation=='sync':s['streams']['rgb']['stamp_ns']-=40_000_000
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):stationary_action_feedback(s,state,now,'current',current_generation=0)
    def scene(self,shift=0,scale=1.):
        hsv=np.zeros((240,320,3),np.uint8);hsv[60:160,40+shift:40+shift+int(100*scale)]=[99,180,180]
        hsv[20:30,280:300]=[99,180,180]
        return cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    def test_shape_change_that_previously_failed_can_continue(self):
        result=hold_assessment(self.scene(),self.scene(25,.65),dict(hue=99,tolerance=8))
        self.assertEqual(result['state'],'consistent')
    def test_same_colour_background_does_not_replace_missing_object(self):
        old=self.scene();now=old.copy();now[60:160,40:140]=0
        a=hold_assessment(old,now,dict(hue=99,tolerance=8));self.assertEqual(a['state'],'absent')
        tracker=HoldEvidence();tracker.observe(a,1_000_000_000)
        with self.assertRaisesRegex(ValueError,'repeated_hand_frames'):tracker.observe(a,1_200_000_000)
    def test_single_occlusion_is_not_a_hard_stop_and_duplicates_do_not_count(self):
        tracker=HoldEvidence();absent=dict(state='absent')
        tracker.observe(absent,1_000_000_000);tracker.observe(absent,1_000_000_000)
        self.assertEqual(len(tracker.absent),1)
        tracker.observe(dict(state='consistent'),1_100_000_000);self.assertEqual(tracker.absent,[])
    def test_functional_grip_allows_small_opening_change_but_not_open_hand(self):
        holding_feedback([.2,-1.7,0,.129,0,.35],.30)
        with self.assertRaises(ValueError):holding_feedback([.2,-1.7,0,.129,0,.65],.30)
    def test_relaxed_arm_endpoint_cannot_break_unchanged_navigation_handoff(self):
        self.assertTrue(joint_endpoint_reached([.25,-1.675,0,.154,0,.3],[.25,-1.7,0,.129,0,.3]))
        goal=[0,0,0,-np.pi/2,0,.3]
        self.assertFalse(joint_endpoint_reached([0,.025,0,-np.pi/2,0,.3],goal))
        self.assertTrue(joint_endpoint_reached([0,.01,0,-np.pi/2,0,.3],goal))
    def test_adaptive_alignment_does_not_require_four_mm_for_wide_open_hand(self):
        self.assertGreater(alignment_tolerances(.022)[1],.004)
        self.assertLessEqual(alignment_tolerances(.006)[1],.003)
    def test_two_stable_close_samples_replace_four_sample_window(self):
        gate=ClosureEvidence(.65)
        self.assertIsNone(gate.observe(5.9,1_000_000_000,.3,0))
        self.assertIsNone(gate.observe(6.0,1_100_000_000,.3,0))
        self.assertEqual(gate.observe(6.21,1_310_000_000,.307,0)['state'],'contact_candidate')
    def test_empty_close_and_arm_motion_still_cannot_prove_grasp(self):
        gate=ClosureEvidence(.65);gate.observe(6.,1_000_000_000,0.,0.)
        self.assertEqual(gate.observe(6.21,1_210_000_000,0.,0.)['state'],'no_contact')
        with self.assertRaises(ValueError):ClosureEvidence(.65).observe(6.,1_000_000_000,.3,.01)
    def test_uncapped_finite_motion_accepts_long_retreat_and_actual_settling(self):
        np.testing.assert_allclose(endpoint([0,0,0],[-1.2,0,0]),[-1.2,0,0])
        self.assertTrue(adjustment_settled(.12,.08,.01))
        self.assertFalse(adjustment_settled(.001,.001,.03))
    def test_nonfinite_motion_and_large_rotation_still_rejected(self):
        for delta in ([float('nan'),0,0],[0,0,.5]):
            with self.assertRaises(ValueError):endpoint([0,0,0],delta)
    def test_multiple_measured_candidates_are_filtered_by_table_and_reach(self):
        s,_,_,_=fixture();table=dict(axis_aligned_bounds=dict(center=dict(x=3.,y=0.),size=dict(x=1.,y=1.)))
        def model(x):return dict(center_m=[x,0,.5],axis_base=[1,0,0],length_m=.084,width_m=.05,height_m=.022,confidence=.8)
        ds=[dict(name='cigarette',x=x) for x in (.8,2.8,2.7,3.3)]
        with patch('flat_box_geometry.estimate',side_effect=lambda _r,_d,_s,d:model(d['x'])):
            chosen,report=select_source(None,None,s,ds,table)
        self.assertEqual(chosen['center_m'][0],2.7);self.assertEqual(report['eligible_candidates'],2)
        self.assertIn('candidate_requires_base_inside_table',report['rejected'])
    def rows(self,count=2):
        s,m,p,t=fixture();p['outcome_profile']='outcome_v1';rows=[]
        for i in range(count):
            ss,mm,tt=copy.deepcopy((s,m,t));stamp=1_000_000_000+i*200_000_000
            ss['streams']['depth']['stamp_ns']=stamp;ss['tracking_transforms']['map_from_base_at_depth']['stamp_ns']=stamp
            mm['stamp_ns']=stamp;tt['source_depth_stamp_ns']=stamp;rows.append(dict(snapshot=ss,measurement=mm,plane=tt))
        return rows,p
    def test_two_real_placed_frames_succeed_even_if_cleanup_is_incomplete(self):
        rows,p=self.rows();proof=verify_placed(rows,p,[.2,-1.7,0,.129,0,.65],cleanup_completed=False)
        self.assertTrue(proof['placement_success_verified']);self.assertFalse(proof['cleanup_completed'])
    def test_one_frame_or_object_above_table_is_not_success(self):
        rows,p=self.rows()
        with self.assertRaises(ValueError):verify_placed(rows[:1],p,[0,0,0,-1.57,0,.65],cleanup_completed=True)
        rows[1]['measurement']['center_m'][2]+=.08
        with self.assertRaises(ValueError):verify_placed(rows,p,[0,0,0,-1.57,0,.65],cleanup_completed=True)
    def test_new_edge_policy_is_explicit_and_does_not_reinterpret_old_policy(self):
        _,_,p,_=fixture();legacy=support_from_plan(p);new=copy.deepcopy(p)
        new['table_edge_policy'].update(version=2,profile='outcome_v1',minimum_outer_gap_m=.02)
        relaxed=support_from_plan(new)
        examples=[x for x in np.arange(.27,.4,.002) if relaxed.query([x,0],.049)['safe'] and not legacy.query([x,0],.049)['safe']]
        self.assertTrue(examples);self.assertFalse(relaxed.query([.2,0],.049)['safe'])
        new['table_edge_policy']['minimum_outer_gap_m']=0
        with self.assertRaises(ValueError):support_from_plan(new)
    def test_occlusion_fallback_is_integrated_but_cannot_authorize_blind_far_approach(self):
        args=(None,None,{},[.7,0,.5],[1,0,0],{},{} )
        with patch('flat_box_geometry.measure_near',side_effect=ValueError('flat_box_top_too_occluded_to_center')),patch('flat_box_partial_top.measure',side_effect=ValueError('edge_hidden')),patch('flat_box_partial_top.locked_patch',return_value=dict(position_xy_is_prediction=True)) as locked:
            with self.assertRaises(ValueError):observe_known(*args,allow_locked=False)
            self.assertFalse(locked.called)
            self.assertTrue(observe_known(*args,allow_locked=True)['position_xy_is_prediction'])

class RuntimeIntegrationTest(unittest.TestCase):
    def test_noop_arm_move_uses_current_feedback_instead_of_exact_past_pose(self):
        from outcome_transfer_runtime import OutcomeRuntime
        class Runtime(OutcomeRuntime):
            def state(self):return dict(joints=[.25,-1.675,0,.154,0,.3])
            def record(self,event):self.event=event
        runtime=Runtime();result=runtime.move_joints([.25,-1.7,0,.129,0,.3],'already_close_enough')
        self.assertTrue(result['command_skipped'])
    def test_all_box_adjustments_use_uncapped_child_without_parent_time_cap(self):
        from outcome_transfer_runtime import OutcomeBackend
        backend=OutcomeBackend.__new__(OutcomeBackend);backend.output=Path('/tmp');backend.config={}
        calls=[];backend.command=lambda *a:calls.append(a)
        backend.adjust('large-retreat',[0,0,0],[-1.,0,0],[.2,-1.7,0,.129,0],.65)
        self.assertTrue(str(calls[0][0][0]).endswith('outcome_finite_adjustment.py'));self.assertIsNone(calls[0][-1])
if __name__=='__main__':unittest.main()
