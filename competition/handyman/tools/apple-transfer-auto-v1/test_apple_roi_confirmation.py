import copy
import unittest
from unittest.mock import patch
from unittest.mock import Mock
from types import SimpleNamespace
import numpy as np

from apple_roi_confirmation import proposal_plan,assess,identity_evidence
from apple_tracking_observer import TrackingObserver
from apple_inference_worker import confirm_near_grasp
from verify_near_grasp_roi_readonly import check_prepared_snapshot


def frame(stamp=1_000_000_000,confidence=.246):
    candidate=dict(stamp_ns=stamp,frame_id='base_link',center_m=[.706,.0715,.539],
                   radius_m=.03,fit_residual_p95_m=.0075)
    detection=dict(name='apple',confidence=confidence,box=[265.7,335.4,310.,380.6])
    proposal=dict(candidate=candidate,detection=detection,support=dict(support_consistent=True))
    result=dict(tracking_proposals=[proposal],detections=[copy.deepcopy(detection)],model_sha256='a'*64,
                timing=dict(freshness_passed=True))
    snapshot=dict(tracking_epoch='epoch',streams=dict(rgb=dict(stamp_ns=stamp,width=640,height=480),
                  depth=dict(stamp_ns=stamp,width=640,height=480)),
                  tracking_transforms=dict(odom_from_base_at_depth=dict(stamp_ns=stamp,
                  source_frame='base_link',target_frame='odom',translation=[0,0,0],quaternion=[0,0,0,1])))
    return result,snapshot


def proof(result,snapshot,confidence=.982,**changes):
    plan=proposal_plan(result['tracking_proposals'],snapshot,result['detections'])
    detection=dict(name='apple',confidence=confidence,box=[265.4,335.1,310.2,378.2]);detection.update(changes)
    value=assess(plan,[detection])
    value['model_sha256']=result['model_sha256']
    return value


class RoiPlanTest(unittest.TestCase):
    def setUp(self):self.result,self.snapshot=frame()
    def plan(self):return proposal_plan(self.result['tracking_proposals'],self.snapshot,self.result['detections'])
    def test_one_fixed_crop_no_geometry_or_detector_score_mutation(self):
        before=copy.deepcopy(self.result);plan=self.plan()
        self.assertTrue(plan['eligible']);self.assertEqual(plan['roi'],[127,160,447,480])
        self.assertEqual(plan['input_size'],640);self.assertEqual(plan['confidence_threshold'],.8)
        self.assertEqual(self.result,before)
    def test_high_full_frame_needs_no_extra_inference(self):
        self.result['tracking_proposals'][0]['detection']['confidence']=.8
        self.assertEqual(self.plan()['reason'],'full_frame_confirmation_already_available')
    def test_multiple_or_unmeasurable_proposals_cannot_request_crop(self):
        self.result['tracking_proposals'].append(dict(rejected_reason='bad_depth'))
        self.assertFalse(self.plan()['eligible'])
    def test_no_box_no_detection_no_crop(self):
        self.result['tracking_proposals']=[]
        self.assertFalse(self.plan()['eligible'])
    def test_not_near_or_bad_geometry(self):
        for key,value in [('center_m',[1.2,0,.5]),('center_m',[.7,.5,.5]),('center_m',[.7,0,.9]),
                          ('radius_m',.2),('fit_residual_p95_m',.02),('frame_id','map')]:
            result,snapshot=frame();result['tracking_proposals'][0]['candidate'][key]=value
            with self.subTest(key=key,value=value):
                self.assertFalse(proposal_plan(result['tracking_proposals'],snapshot)['eligible'])
    def test_support_contact_is_not_a_classification_precondition(self):
        self.result['tracking_proposals'][0]['support']['support_consistent']=False
        self.assertTrue(self.plan()['eligible'])
    def test_same_frame_and30ms_alignment_required(self):
        self.snapshot['streams']['rgb']['stamp_ns']-=30_000_001
        self.assertEqual(self.plan()['reason'],'same_frame_rgbd_required')
        self.snapshot['streams']['rgb']['stamp_ns']+=30_000_001
        self.result['tracking_proposals'][0]['candidate']['stamp_ns']-=1
        self.assertFalse(self.plan()['eligible'])
    def test_clipped_box_shape_and_nan_decline(self):
        for box in ([0,300,50,350],[100,100,90,140],[float('nan'),100,150,140]):
            self.result,self.snapshot=frame();self.result['tracking_proposals'][0]['detection']['box']=box
            self.assertFalse(self.plan()['eligible'])
        self.result,self.snapshot=frame();self.snapshot['streams']['rgb']['width']=1280
        self.assertFalse(self.plan()['eligible'])
    def test_high_confidence_conflicting_full_frame_class_cannot_be_hidden(self):
        self.result['detections'].append(dict(name='toy_car',confidence=.9,box=[265,335,310,380]))
        self.assertEqual(self.plan()['reason'],'full_frame_conflicting_class')


class RoiVerdictTest(unittest.TestCase):
    def setUp(self):self.result,self.snapshot=frame()
    def test_confirmed_evidence_bound_to_same_model_frame_and_box(self):
        self.result['near_grasp_roi_confirmation']=proof(self.result,self.snapshot)
        value=identity_evidence(self.result,self.snapshot)
        self.assertEqual(value['confidence'],.982)
        self.assertEqual(value['full_frame_confidence'],.246)
        self.assertFalse(value['proof']['motion_authorized'])
    def test_disabled_or_missing_proof_is_not_evidence(self):
        self.assertIsNone(identity_evidence(self.result,self.snapshot))
    def test_low_score_does_not_confirm(self):
        self.result['near_grasp_roi_confirmation']=proof(self.result,self.snapshot,.79)
        self.assertIsNone(identity_evidence(self.result,self.snapshot))
    def test_no_apple_does_not_confirm(self):
        plan=proposal_plan(self.result['tracking_proposals'],self.snapshot)
        self.result['near_grasp_roi_confirmation']=dict(assess(plan,[]),model_sha256='a'*64)
        self.assertIsNone(identity_evidence(self.result,self.snapshot))
    def test_far_box_does_not_resurrect_different_target(self):
        self.result['near_grasp_roi_confirmation']=proof(self.result,self.snapshot,box=[150,200,190,240])
        with self.assertRaisesRegex(ValueError,'roi_target_mismatch'):
            identity_evidence(self.result,self.snapshot)
    def test_two_apples_even_low_confidence_twins_are_ambiguous(self):
        value=proof(self.result,self.snapshot);local=value['detections']
        local.append(dict(local[0],confidence=.11))
        self.result['near_grasp_roi_confirmation']=dict(assess(value['plan'],local),model_sha256='a'*64)
        with self.assertRaisesRegex(ValueError,'roi_ambiguous_apple'):
            identity_evidence(self.result,self.snapshot)
    def test_crop_class_conflict_blocks_old_identity_too(self):
        value=proof(self.result,self.snapshot);local=value['detections']
        local.append(dict(local[0],name='toy_car',confidence=.9))
        self.result['near_grasp_roi_confirmation']=dict(assess(value['plan'],local),model_sha256='a'*64)
        with self.assertRaisesRegex(ValueError,'roi_conflicting_class'):
            identity_evidence(self.result,self.snapshot)
    def test_old_frame_or_changed_source_box_rejected(self):
        for mutate in ('stamp','box'):
            result,snapshot=frame();result['near_grasp_roi_confirmation']=proof(result,snapshot)
            if mutate=='stamp':snapshot['streams']['rgb']['stamp_ns']+=1
            else:result['tracking_proposals'][0]['detection']['box'][0]-=1
            with self.subTest(mutate=mutate),self.assertRaisesRegex(ValueError,'roi_source_binding_mismatch'):
                identity_evidence(result,snapshot)
    def test_fake_verdict_model_or_out_of_crop_coordinates_rejected(self):
        for mutate in ('verdict','model','coordinates'):
            result,snapshot=frame();value=proof(result,snapshot)
            if mutate=='verdict':value['confidence']=.999
            elif mutate=='model':value['model_sha256']='b'*64
            else:value['detections'][0]['box'][0]=0
            result['near_grasp_roi_confirmation']=value
            with self.subTest(mutate=mutate),self.assertRaises(ValueError):identity_evidence(result,snapshot)


class RoiTrackingTest(unittest.TestCase):
    def setUp(self):
        self.observer=TrackingObserver('task','epoch',{},'bedroom',near_grasp_roi=True)
        self.room=patch('apple_tracking_observer.room_check',return_value=dict(room_confirmed=True))
        self.room.start();self.addCleanup(self.room.stop)
    def observe(self,stamp,*,roi=True):
        result,snapshot=frame(stamp)
        if roi:result['near_grasp_roi_confirmation']=proof(result,snapshot)
        return self.observer.observe(result,snapshot,stamp+100_000_000)
    def test_three_new_roi_frames_required_with_original_geometry(self):
        first=self.observe(1_000_000_000);second=self.observe(1_100_000_000);third=self.observe(1_200_000_000)
        self.assertFalse(first['identity_confirmed']);self.assertFalse(second['identity_confirmed'])
        self.assertTrue(third['identity_confirmed']);self.assertFalse(third['motion_authorized'])
        m=third['measurement'];self.assertEqual(m['point_odom_m'],[.706,.0715,.539])
        self.assertEqual(m['full_frame_confidence'],.246);self.assertEqual(m['confidence'],.982)
    def test_gap_reentry_uses_new_bound_strong_roi_and_same_geometry(self):
        for stamp in (1_000_000_000,1_100_000_000,1_200_000_000):self.observe(stamp)
        result=self.observe(2_309_189_623)
        self.assertTrue(result['identity_confirmed'])
        self.assertTrue(result['gap_reconfirmed_with_current_classification'])

    def test_gap_weak_reentry_without_current_roi_still_resets(self):
        for stamp in (1_000_000_000,1_100_000_000,1_200_000_000):self.observe(stamp)
        result=self.observe(2_309_189_623,roi=False)
        self.assertEqual(result['reason'],'identity_reconfirmation_required_after_gap')
        for stamp in (2_410_000_000,2_510_000_000):self.assertFalse(self.observe(stamp)['identity_confirmed'])
        self.assertTrue(self.observe(2_610_000_000)['identity_confirmed'])
    def test_opt_in_required_and_no_weak_seed_without_proof(self):
        self.assertFalse(self.observe(1_000_000_000,roi=False)['identity_confirmed'])
        self.observer=TrackingObserver('task','epoch',{},'bedroom')
        for stamp in (1_100_000_000,1_200_000_000,1_300_000_000):
            self.assertFalse(self.observe(stamp)['identity_confirmed'])
    def test_roi_cannot_bypass_stale_pixels_room_or_epoch(self):
        for mutation in ('stale','room','epoch'):
            result,snapshot=frame();result['near_grasp_roi_confirmation']=proof(result,snapshot)
            if mutation=='epoch':snapshot['tracking_epoch']='other'
            now=1_500_000_000 if mutation=='stale' else 1_100_000_000
            with self.subTest(mutation=mutation):
                if mutation=='room':
                    with patch('apple_tracking_observer.room_check',return_value=dict(room_confirmed=False)):
                        value=self.observer.observe(result,snapshot,now)
                else:value=self.observer.observe(result,snapshot,now)
                self.assertFalse(value['identity_confirmed'])


class WorkerRoiIntegrationTest(unittest.TestCase):
    def test_exact_one_bgr_crop_original_boxes_and_geometry_preserved(self):
        result,snapshot=frame();before=copy.deepcopy(result)
        rgb=np.zeros((480,640,3),dtype=np.uint8);rgb[:,:,0]=255
        tensor=lambda values:SimpleNamespace(cpu=lambda:SimpleNamespace(tolist=lambda:values))
        boxes=SimpleNamespace(xyxy=tensor([[138.4,175.1,183.2,218.2]]),cls=tensor([0]),conf=tensor([.982]))
        model=Mock(names={0:'apple'});model.names={0:'apple'}
        model.predict.return_value=[SimpleNamespace(boxes=boxes)]
        value=confirm_near_grasp(model,rgb,snapshot,result['tracking_proposals'],result['detections'],'a'*64,'cpu')
        self.assertEqual(value['status'],'confirmed');self.assertEqual(value['detections'][0]['box'],[265.4,335.1,310.2,378.2])
        self.assertEqual(model.predict.call_count,1)
        crop=model.predict.call_args.args[0]
        self.assertEqual(crop.shape,(320,320,3));np.testing.assert_array_equal(crop[0,0],[0,0,255])
        self.assertEqual(model.predict.call_args.kwargs['imgsz'],640)
        np.testing.assert_array_equal(rgb[0,0],[255,0,0]);self.assertEqual(result,before)
    def test_high_full_frame_or_ambiguity_never_runs_crop(self):
        model=Mock()
        for confidence,ambiguous in ((.9,False),(.246,True)):
            result,snapshot=frame(confidence=confidence)
            if ambiguous:result['tracking_proposals'].append(dict(rejected_reason='missing_depth'))
            value=confirm_near_grasp(model,np.zeros((480,640,3),dtype=np.uint8),snapshot,
                result['tracking_proposals'],result['detections'],'a'*64,'cpu')
            self.assertEqual(value['status'],'skipped')
        model.predict.assert_not_called()


class PreparedReadonlyPoseTest(unittest.TestCase):
    def test_prepared_pose_does_not_fake_empty_tucked_preflight(self):
        _,snapshot=frame();snapshot['tracking_transforms']['map_from_base_at_depth']=dict(
            snapshot['tracking_transforms']['odom_from_base_at_depth'],target_frame='map')
        snapshot['joints']=dict(arm_lift_joint=.2525,arm_flex_joint=-1.7,arm_roll_joint=0,
            wrist_flex_joint=.1292036732051034,wrist_roll_joint=0,hand_motor_joint=.65,
            head_pan_joint=0,head_tilt_joint=-.5)
        expected=dict(arm=[.2525,-1.7,0,.1292036732051034,0],holding_object=False,
                      hand_motor=.65,base_map_pose=[0,0,0])
        self.assertEqual(check_prepared_snapshot(snapshot,expected),[0,0,0])
        for key,value in [('hand_motor_joint',.4),('arm_flex_joint',0),('head_tilt_joint',0)]:
            changed=copy.deepcopy(snapshot);changed['joints'][key]=value
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'prepared_open_pose_changed'):
                check_prepared_snapshot(changed,expected)
        changed=copy.deepcopy(snapshot)
        changed['tracking_transforms']['map_from_base_at_depth']['translation'][0]=.1
        with self.assertRaisesRegex(ValueError,'base_moved_from_reviewed_prepared_pose'):
            check_prepared_snapshot(changed,expected)
        with self.assertRaisesRegex(ValueError,'reviewed_prepared_empty_reference_required'):
            check_prepared_snapshot(snapshot,dict(expected,holding_object=True))


if __name__=='__main__':unittest.main()
