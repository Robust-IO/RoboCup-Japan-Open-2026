"""Synthetic RGBD tests: inherited identity is NOT a fabricated detection."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

from apple_approach_lease import measured_target, read_latest, validate_closure
from apple_tracking_observer import TrackingObserver
from latest_capture_index import publish
from near_apple_geometry import check_gap, fit_current, recorded_candidate
from hand_camera_ray import rotation
from verify_apple_lift_sequence import measured


def scene(stamp, *, base_x=0., centers=((.78, .078, .54),), radius=.03):
    h,w=240,320;v,u=np.indices((h,w));rays=np.stack(((u-160)/300,(v-120)/300,np.ones((h,w))),-1)
    camera=np.array([0.,.078,.53]);quat=[.5,-.5,.5,-.5];orient=rotation(quat)
    depth=np.full((h,w),np.inf);a=(rays*rays).sum(2)
    for point in centers:
        relative=orient.T@(np.asarray(point)-[base_x,0,0]-camera)
        b=-2*(rays*relative).sum(2);c=relative@relative-radius**2;disc=b*b-4*a*c
        hit=disc>0;z=np.full((h,w),np.inf);z[hit]=(-b[hit]-np.sqrt(disc[hit]))/(2*a[hit])
        z[z<=0]=np.inf;depth=np.minimum(depth,z)
    rgb=np.zeros((h,w,3),np.uint8);visible=np.isfinite(depth);rgb[visible]=[230,30,50];depth[~visible]=0
    tf=dict(stamp_ns=stamp,source_frame='base_link',target_frame='odom',translation=[base_x,0,0],quaternion=[0,0,0,1])
    snap=dict(tracking_epoch='epoch',streams=dict(rgb=dict(stamp_ns=stamp,width=w,height=h),
        depth=dict(stamp_ns=stamp,width=w,height=h)),rgb_k=[300,0,160,0,300,120,0,0,1],
        base_from_rgb=dict(translation=camera.tolist(),quaternion=quat),joint_stamp_ns=stamp,
        joints=dict(arm_lift_joint=.2528,arm_flex_joint=-1.7,arm_roll_joint=0.,
                    wrist_flex_joint=.1292036732051034,wrist_roll_joint=0.,hand_motor_joint=.65),
        tracking_transforms=dict(odom_from_base_at_depth=tf))
    return rgb,depth,snap


class GeometryHandoffTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.observer=TrackingObserver('task','epoch',{},'bedroom',near_geometry=True)
        self.room=patch('apple_tracking_observer.room_check',return_value=dict(room_confirmed=True,target_room='bedroom'))
        self.room.start();self.addCleanup(self.room.stop);self.addCleanup(self.temp.cleanup)
        self.stamp=1_000_000_000

    def seed(self,count=3):
        for i in range(count):
            self.stamp+=100_000_000;rgb,depth,snap=scene(self.stamp,base_x=-.3)
            candidate=dict(center_m=[1.08,.078,.54],radius_m=.03,fit_residual_p95_m=.002,
                           stamp_ns=self.stamp,frame_id='base_link',source='rgbd')
            detection=dict(name='apple',confidence=.95,box=[148,108,172,132])
            result=dict(status='candidate_only',timing=dict(freshness_passed=True),detections=[detection],
                tracking_proposals=[dict(candidate=candidate,detection=detection,support=dict(support_consistent=True))],
                model_sha256='a'*64,tracking_epoch='epoch',identity_tracking_task_id='task')
            capture=self.root/f'{i:06d}';capture.mkdir()
            evidence=self.observer.observe(result,snap,self.stamp+10_000_000,capture=capture)
            result['identity_tracking']=evidence
            (capture/'observation.json').write_text(json.dumps(result));(capture/'snapshot.json').write_text(json.dumps(snap))
        return evidence

    def near(self,**kwargs):
        self.stamp+=100_000_000;rgb,depth,snap=scene(self.stamp,**kwargs)
        return self.observer.observe_geometry(rgb,depth,snap,self.stamp+10_000_000),rgb,depth,snap

    def test_three_seeds_then_no_detector_needed_and_base_motion_accounted_for(self):
        self.assertTrue(self.seed()['identity_confirmed'])
        value,rgb,depth,snap=self.near();identity=value['identity_tracking']
        self.assertEqual(identity['status'],'near_geometry_tracked');self.assertTrue(identity['identity_confirmed'])
        np.testing.assert_allclose(identity['measurement']['point_odom_m'],[.78,.078,.54],atol=1e-5)
        self.assertEqual(value['detections'],[]);self.assertFalse(value['detector_ran'])
        self.assertIsNone(identity['measurement']['confidence']);self.assertFalse(identity['identity_reclassified'])
        self.assertFalse(identity['motion_authorized']);self.assertFalse(identity['current_geometry_candidate']['identity_confirmed'])

    def test_one_or_two_frames_cannot_lock(self):
        self.seed(2);self.assertIsNone(self.near()[0])

    def test_far_target_does_not_enter_geometry_mode(self):
        self.seed();self.assertIsNone(self.near(base_x=-.3)[0])

    def test_current_depth_changes_position_not_frozen_prediction(self):
        self.seed();value,*_=self.near(centers=((.79,.078,.54),))
        self.assertAlmostEqual(value['identity_tracking']['measurement']['point_odom_m'][0],.79,places=5)

    def test_no_accumulating_drift(self):
        self.seed()
        for x in (.79,.80):self.assertEqual(self.near(centers=((x,.078,.54),))[0]['status'],'geometry_only')
        self.assertEqual(self.near(centers=((.82,.078,.54),))[0]['status'],'rejected')
        self.assertIsNone(self.observer.geometry.anchor)

    def test_missing_depth_or_flat_red_patch_cannot_supply_old_center(self):
        for flat in (False,True):
            self.setUp();self.seed();self.stamp+=100_000_000;rgb,depth,snap=scene(self.stamp)
            depth[depth>0]=.75 if flat else 0
            value=self.observer.observe_geometry(rgb,depth,snap,self.stamp+10_000_000)
            self.assertEqual(value['status'],'rejected');self.assertFalse(value['identity_tracking']['identity_confirmed'])

    def test_two_nearby_surfaces_are_not_averaged(self):
        self.seed();value,*_=self.near(centers=((.78,.045,.54),(.78,.11,.54)),radius=.02)
        self.assertEqual(value['status'],'rejected')

    def test_stale_rgb_joint_epoch_and_reversed_frames_invalidate_active_handoff(self):
        for mutation in ('rgb','joint','epoch','reversed','hand','arm','tf'):
            self.setUp();self.seed();self.near();self.stamp+=100_000_000;rgb,depth,snap=scene(self.stamp)
            if mutation=='rgb':snap['streams']['rgb']['stamp_ns']-=500_000_000
            elif mutation=='joint':snap['joint_stamp_ns']-=500_000_000
            elif mutation=='epoch':snap['tracking_epoch']='old'
            elif mutation=='reversed':self.observer.geometry.last_stamp=self.stamp
            elif mutation=='hand':snap['joints']['hand_motor_joint']=.4
            elif mutation=='arm':snap['joints']['arm_lift_joint']+=.1
            else:snap['tracking_transforms']['odom_from_base_at_depth']['stamp_ns']-=1
            with self.subTest(mutation=mutation):
                value=self.observer.observe_geometry(rgb,depth,snap,self.stamp+10_000_000)
                self.assertEqual(value['status'],'rejected');self.assertFalse(self.observer.geometry.active)

    def test_room_boundary_is_still_checked_for_current_measured_point(self):
        self.seed()
        with patch('apple_tracking_observer.room_check',return_value=dict(room_confirmed=False)):
            self.assertEqual(self.near()[0]['identity_tracking']['reason'],'near_geometry_room_unverified')

    def write_near(self,**kwargs):
        value,rgb,depth,snap=self.near(**kwargs);value.update(tracking_epoch='epoch',timing=dict(freshness_passed=True))
        capture=self.root/'000003';capture.mkdir()
        (capture/'observation.json').write_text(json.dumps(value));(capture/'snapshot.json').write_text(json.dumps(snap))
        Image.fromarray(rgb).save(capture/'rgb.png');np.save(capture/'depth_m.npy',depth)
        publish(self.root,capture.name,'epoch',observation=True)
        return value,rgb,depth,snap,capture

    def test_closure_reads_real_seed_files_and_refits_current_depth_without_a_yolo_box(self):
        self.seed();value,rgb,depth,snap,capture=self.write_near(base_x=.076)
        evidence=read_latest(self.root,closure=True,now_ns=self.stamp+10_000_000,epoch='epoch',
            task_id='task',current_odom=[.076,0,0],palm_height=.54,expected_room='bedroom')
        np.testing.assert_allclose(evidence['target_base_m'],[.704,.078,.54],atol=1e-5)
        obs,s,candidate=measured(capture)
        self.assertIsNone(candidate['detector_confidence']);self.assertEqual(obs['detections'],[])

    def test_seed_from_another_recording_or_tampered_class_cannot_close(self):
        self.seed();value,rgb,depth,snap,capture=self.write_near(base_x=.076)
        seedpath=self.root/'000000/observation.json';seed=json.loads(seedpath.read_text())
        seed['tracking_proposals'][0]['detection']['name']='toy_car';seedpath.write_text(json.dumps(seed))
        with self.assertRaises(ValueError):recorded_candidate(value,snap,rgb,depth,directory=self.root)
        value['identity_tracking']['near_geometry_proof']['anchor']['identity_seeds'][0]['capture']='/other/000000'
        with self.assertRaisesRegex(ValueError,'outside_current_recording'):
            recorded_candidate(value,snap,rgb,depth,directory=self.root)

    def test_closure_refit_io_time_counts_against_original_freshness(self):
        self.seed();self.write_near(base_x=.076)
        with patch('apple_approach_lease.time.monotonic_ns',side_effect=[1_000_000_000,1_100_000_000]):
            with self.assertRaisesRegex(ValueError,'expired_or_unsynchronised'):
                read_latest(self.root,closure=True,now_ns=self.stamp+350_000_000,epoch='epoch',
                    task_id='task',current_odom=[.076,0,0],palm_height=.54,expected_room='bedroom')

    def test_saved_center_tamper_or_no_current_depth_cannot_pass_reaudit(self):
        self.seed();value,rgb,depth,snap,capture=self.write_near()
        value['identity_tracking']['measurement']['point_odom_m'][0]+=.01
        with self.assertRaisesRegex(ValueError,'differs_from_current_pixels'):recorded_candidate(value,snap,rgb,depth)
        with self.assertRaises(ValueError):recorded_candidate(value,snap,rgb,depth*0)

    def test_replaying_old_capture_cannot_renew_motion_lease(self):
        self.seed();value,rgb,depth,snap,capture=self.write_near()
        with self.assertRaisesRegex(ValueError,'expired_or_unsynchronised'):
            measured_target(value,snap,now_ns=self.stamp+500_000_000,epoch='epoch',task_id='task',current_odom=[0,0,0],expected_room='bedroom')

    def test_gap_is_strict_finite_numeric(self):
        for value in (None,True,0,-.1,.5,float('nan'),float('inf'),'0.15'):
            with self.subTest(value=value),self.assertRaises(ValueError):check_gap(value)

    def test_postlift_independent_audit_accepts_geometry_preclose_and_no_postlift_detector(self):
        from audit_preclose_identity_lift_follow import audit
        self.seed();value,rgb,depth,snap,capture=self.write_near(base_x=.076)
        def palm(snapshot,lift):
            snapshot['base_from_hand']=dict(translation=[.6892952185,.078,.2771978151685961+lift],quaternion=[0,0,0,1])
            snapshot['palm_from_hand']=dict(translation=[0,0,0],quaternion=[0,0,0,1])
        palm(snap,.2528);(capture/'snapshot.json').write_text(json.dumps(snap))
        before_stamp=self.stamp
        for i in range(4,7):
            stamp=before_stamp+(i-3)*500_000_000
            image,z,s=scene(stamp,base_x=.076,centers=((.78,.078,.64),))
            s['joints']['arm_lift_joint']=.3528;s['joints']['hand_motor_joint']=.4;palm(s,.3528)
            frame=self.root/f'{i:06d}';frame.mkdir()
            Image.fromarray(image).save(frame/'rgb.png');np.save(frame/'depth_m.npy',z)
            (frame/'snapshot.json').write_text(json.dumps(s))
            (frame/'observation.json').write_text(json.dumps(dict(detections=[],tracking_epoch='epoch',identity_tracking_task_id='task')))
        trial=self.root/'close-lift';trial.mkdir()
        report=dict(status='visual_verification_inconclusive',grasp_success_verified=False,
            observer_owned=False,owned_near_grasp_roi_enabled=True,
            observation_directory=str(self.root),tracking_epoch='epoch',tracking_task_id='task',
            close=dict(status='passed',grasp_closure_commanded=True,
                closure_evidence=dict(state='contact_candidate',observed_opening_rad=.4,stable_samples=6,
                    stable_duration_s=.7,closing_travel_rad=.25),
                closure_identity_evidence=dict(evidence=str(capture/'observation.json'),stamp_ns=before_stamp,age_s=.1)),
            lift=dict(status='passed'),lift_completed_at_ns=before_stamp+250_000_000,
            verification_window_end_ns=before_stamp+1_500_000_000,
            verification_last_complete_capture=str(self.root/'000006'))
        (trial/'result.json').write_text(json.dumps(report))
        result=audit(trial,expected_recording=self.root,expected_epoch='epoch',expected_task_id='task')
        self.assertTrue(result['grasp_success_verified']);self.assertEqual(result['accepted_after_frames'],3)
        self.assertTrue(all(r['confidence'] is None for r in result['rows']))
        self.assertFalse(result['detector_confidence_invented'])


if __name__=='__main__':unittest.main()
