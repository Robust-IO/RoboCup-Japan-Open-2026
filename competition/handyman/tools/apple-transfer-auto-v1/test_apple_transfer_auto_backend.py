"""Exercise real adapter wiring with command calls intercepted; never imports ROS."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch
import copy
from apple_transfer_auto_plan import validate_config

from apple_transfer_auto_backend import LiveBackend

CONFIG = json.loads((Path(__file__).parent/'config/apple_transfer_auto_v1.json').read_text())


class AdapterWiringTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.backend = LiveBackend.__new__(LiveBackend)
        b = self.backend
        b.config = CONFIG
        b.code = Path('/NOT_A_LIVE_INSTALL')
        b.output = Path(self.temp.name)
        b.task_id = 'test-task'
        b.holding = .415
        b.place_plan = dict(palm_height_m=.486)
        b.place_plan_path = b.output/'plan.json'
        b.recording = b.output/'recording'
        b.epoch = 'test-epoch'
        self.snapshot = dict(streams=dict(depth=dict(stamp_ns=1)), tracking_transforms=dict(map_from_base_at_depth=dict(
            translation=[2.1,2.9,0], quaternion=[0,0,0,1], stamp_ns=1, source_frame='base_link', target_frame='map')))
        self.snapshot['joints']=dict(arm_lift_joint=.29,arm_flex_joint=-1.7,arm_roll_joint=0.,wrist_flex_joint=.1292,wrist_roll_joint=0.,hand_motor_joint=.62)
        b.latest = Mock(return_value=(b.output,self.snapshot,None,None))
    def tearDown(self): self.temp.cleanup()
    def test_navigation_only_checks_existing_precision(self):
        b = self.backend
        b.route = Mock(return_value=[[6.99,-6.24,3.14]])
        b.command = Mock(return_value=dict(status='arrived',nav_reset=True,parameters_changed=True,
            goal_precision_changed=False,speed_parameters_restored=True,grip_commands_sent=False))
        b.navigate('bedroom','living_room',[2.05,2.55,1.57],.415,'nav')
        args = b.command.call_args.args[0]
        self.assertIn('--verify-current-precision',args)
        self.assertNotIn('--precise',args)
        self.assertIn('--waypoint',args)
        self.assertIn('--motion-profile',args)
        self.assertEqual(args[args.index('--motion-profile')+1],'carry_fast_v1')
        self.assertEqual(b.config['motion_profile'],'balanced_v1')
    def test_unrestored_speed_limits_cannot_count_as_completed_navigation(self):
        b = self.backend
        b.route = Mock(return_value=[])
        b.command = Mock(return_value=dict(status='arrived',nav_reset=True,parameters_changed=True,
            goal_precision_changed=False,speed_parameters_restored=False,grip_commands_sent=False))
        with self.assertRaises(ValueError): b.navigate('bedroom','bedroom',[9.9,-6.6,0],0.,'nav')
    def test_navigation_missing_reset_blocks_next_stage(self):
        b = self.backend
        b.route = Mock(return_value=[])
        b.command = Mock(return_value=dict(status='arrived',nav_reset=False,parameters_changed=False,grip_commands_sent=False))
        with self.assertRaises(ValueError): b.navigate('bedroom','bedroom',[9.9,-6.6,0],0.,'nav')
    def test_release_passes_live_lease_to_publish_boundary(self):
        b = self.backend
        b.arm = Mock(return_value=(dict(release_geometry_evidence={'center_map':[1.97,3.6,.49]}),'test-evidence'))
        b.release()
        flags = b.arm.call_args.args[2]
        for flag in ('--release-recording','--release-plan','--release-epoch','--release-task-id'):
            self.assertIn(flag,flags)
        self.assertEqual(b.prior_release['center_map'],[1.97,3.6,.49])
    def test_release_without_boundary_evidence_not_accepted(self):
        b = self.backend
        b.arm = Mock(return_value=(dict(status='passed'),'test-evidence'))
        with self.assertRaises(ValueError): b.release()

    def test_single_placement_flag_is_explicit_not_applied_to_retreat(self):
        b=self.backend;b.command=Mock(return_value=dict(status='endpoint_verified'))
        b.adjust('single',[2.,2.5,1.57],[.473,-.028,0],[.2168,-1.7,0,.1292036732051034,0],.415,single_placement_approach=True)
        self.assertIn('--single-placement-approach',b.command.call_args.args[0])
        self.assertIsNone(b.command.call_args.args[4])
        b.adjust('retreat',[2.,2.5,1.57],[-.25,0,0],[.2968,-1.7,0,.1292036732051034,0],.65)
        self.assertNotIn('--single-placement-approach',b.command.call_args.args[0])
        self.assertEqual(b.command.call_args.args[4],25)

    def test_config_single_mode_is_default_and_has_matching_limit(self):
        self.assertEqual(CONFIG['policy']['placement_approach_mode'],'single')
        self.assertIsNone(CONFIG['policy']['max_placement_step_m'])
        bad=copy.deepcopy(CONFIG);bad['policy']['max_placement_step_m']=.4
        with self.assertRaises(ValueError):validate_config(bad)
        legacy=copy.deepcopy(CONFIG);legacy['policy']['placement_approach_mode']='segmented';legacy['policy']['max_placement_step_m']=.4
        validate_config(legacy)

    def test_unknown_approach_mode_rejected(self):
        bad=copy.deepcopy(CONFIG);bad['policy']['placement_approach_mode']='fast'
        with self.assertRaisesRegex(ValueError,'invalid_placement_approach_mode'):validate_config(bad)

    def single_approach_fixture(self,endpoint_x=1.25):
        b=self.backend
        first=copy.deepcopy(self.snapshot);last=copy.deepcopy(self.snapshot)
        first['tracking_transforms']['map_from_base_at_depth']['translation']=[0.,0.,0.]
        last['tracking_transforms']['map_from_base_at_depth']['translation']=[endpoint_x,0.,0.]
        b.place_plan=dict(palm_height_m=.486,release_xy_map_m=[1.95,.078])
        b.latest=Mock(side_effect=[(b.output,first,None,None),(b.output,last,None,None)])
        b.held_measurement=Mock(return_value=dict(center_base=[.7,.078,.5]))
        b.adjust=Mock(return_value='SYNTHETIC_TEST_ONLY')
        return b

    def test_full_one_point_two_five_metre_approach_is_one_command_not_capped_or_split(self):
        b=self.single_approach_fixture()
        self.assertTrue(b.approach_placement()['verified'])
        self.assertEqual(b.adjust.call_count,1)
        self.assertAlmostEqual(b.adjust.call_args.args[2][0],1.25)
        self.assertTrue(b.adjust.call_args.kwargs['single_placement_approach'])
        result=json.loads((b.output/'placement-approach-sequence.json').read_text())
        self.assertEqual(len(result['steps']),1)
        self.assertAlmostEqual(result['total_translation_m'],1.25)

    def test_wrong_single_endpoint_never_causes_automatic_second_movement(self):
        b=self.single_approach_fixture(endpoint_x=1.1)
        with self.assertRaisesRegex(ValueError,'single_placement_endpoint_not_aligned'):b.approach_placement()
        self.assertEqual(b.adjust.call_count,1)

    def test_single_limit_config_is_explicit_null_not_a_replacement_numeric_cap(self):
        for limit in (.6,1.,8.,float('nan'),'none',False):
            config=copy.deepcopy(CONFIG);config['policy']['max_placement_step_m']=limit
            with self.subTest(limit=limit),self.assertRaises(ValueError):validate_config(config)
        config=copy.deepcopy(CONFIG);del config['policy']['max_placement_step_m']
        with self.assertRaises(ValueError):validate_config(config)
    def test_failed_retreat_does_not_retract_arm_or_replay_release(self):
        b = self.backend
        b.arm = Mock(return_value=(dict(status='passed'),'test-evidence'))
        b.adjust = Mock(side_effect=RuntimeError('retreat_failed'))
        b.stop_observer = Mock()
        result=b.withdraw()
        self.assertFalse(result['verified'])
        self.assertTrue(result['deferred_to_object_verification'])
        self.assertTrue(result['cleanup_required'])
        self.assertEqual([c.args[0] for c in b.arm.call_args_list],['place-raise'])
        self.assertEqual(b.adjust.call_count,1)
        b.stop_observer.assert_called_once()
    def test_withdrawal_retreat_preserves_actual_not_nominal_arm_and_hand(self):
        b=self.backend;b.arm=Mock(return_value=(dict(status='passed'),'test-evidence'))
        b.adjust=Mock(return_value='retreat-evidence');b.stop_observer=Mock()
        self.assertTrue(b.withdraw()['verified'])
        self.assertEqual(b.adjust.call_args.args[3],[.29,-1.7,0.,.1292,0.])
        self.assertEqual(b.adjust.call_args.args[4],.62)
    def test_prepare_grasp_passes_newly_measured_height(self):
        b = self.backend
        b.grasp_plan = dict(contact_start_lift_m=.258)
        b.ready = dict(initial_hand_open=True)
        b.arm = Mock(return_value=(dict(status='passed'),'test-evidence'))
        b.prepare_grasp()
        flags = b.arm.call_args.args[2]
        self.assertEqual(flags[flags.index('--white-table-prepare-lift')+1],.258)
        self.assertIn('--prepare-hand-already-open',flags)
    def test_roi_configuration_is_strict_boolean(self):
        value=copy.deepcopy(CONFIG);value['policy']['near_grasp_roi_confirmation']='false'
        with self.assertRaisesRegex(ValueError,'invalid_near_grasp_roi_option'):validate_config(value)

    def test_geometry_handoff_configuration(self):
        self.assertTrue(CONFIG['policy']['near_grasp_geometry_handoff'])
        self.assertEqual(CONFIG['policy']['near_grasp_geometry_gap_m'],.15)
        for value in ('true',1,None):
            config=copy.deepcopy(CONFIG);config['policy']['near_grasp_geometry_handoff']=value
            with self.assertRaises(ValueError):validate_config(config)
        for value in (float('nan'),float('inf'),True,0):
            config=copy.deepcopy(CONFIG);config['policy']['near_grasp_geometry_gap_m']=value
            with self.assertRaises(ValueError):validate_config(config)

    def test_postlift_policy_is_strict_boolean(self):
        value=copy.deepcopy(CONFIG);value['policy']['post_lift_rgbd_follow_verification']='true'
        with self.assertRaisesRegex(ValueError,'invalid_post_lift_rgbd_follow_option'):validate_config(value)

    def test_independent_policy_is_propagated_to_regrasp_child(self):
        b=self.backend;b.grasp_plan=dict(contact_start_lift_m=.252)
        b.command=Mock(return_value=dict(grasp_success_verified=True))
        b.grasp_and_lift()
        self.assertIn('--verify-preclose-rgbd-follow',b.command.call_args.args[0])
        b.config=copy.deepcopy(CONFIG);b.config['policy']['post_lift_rgbd_follow_verification']=False
        b.grasp_and_lift()
        self.assertNotIn('--verify-preclose-rgbd-follow',b.command.call_args.args[0])

    def independent_context(self):
        trial=dict(effective_grasp_success_verified=True,
            close=dict(status='passed',grasp_closure_commanded=True,
                closure_evidence=dict(state='contact_candidate',observed_opening_rad=.4,
                    stable_samples=6,stable_duration_s=.7,closing_travel_rad=.25)),lift=dict(status='passed'))
        capture=self.backend.output/'raw-frame'
        candidate=dict(center_m=[.7,.078,.64],radius_m=.03)
        proof=dict(grasp_success_verified=True,rows=[dict(capture=str(capture),center_measurement=candidate)])
        snapshot=dict(base_from_hand=dict(translation=[.627,.078,.63],quaternion=[0,0,0,1]),
                      palm_from_hand=dict(translation=[0,0,0],quaternion=[0,0,0,1]))
        return trial,proof,snapshot

    def test_independent_grasp_reads_real_candidate_not_a_missing_detector_box(self):
        trial,proof,snapshot=self.independent_context();b=self.backend
        with patch('apple_transfer_auto_backend.read',side_effect=[trial,snapshot]), \
                patch('audit_preclose_identity_lift_follow.audit',return_value=proof) as audit, \
                patch('apple_transfer_auto_backend.measured') as detector:
            self.assertTrue(b.verify_grasp()['verified'])
            detector.assert_not_called()
            self.assertEqual(audit.call_args.kwargs,dict(expected_recording=b.recording,expected_epoch=b.epoch,expected_task_id=b.task_id))
            self.assertAlmostEqual(b.held['center_minus_palm_m'][0],.073)
            self.assertAlmostEqual(b.holding,.4)

    def test_unverified_effective_report_cannot_handoff(self):
        trial,proof,snapshot=self.independent_context();trial['effective_grasp_success_verified']=False
        with patch('apple_transfer_auto_backend.read',return_value=trial), \
                patch('audit_preclose_identity_lift_follow.audit') as audit:
            with self.assertRaisesRegex(ValueError,'independent_lift_evidence_not_verified'):self.backend.verify_grasp()
            audit.assert_not_called()

    def test_failed_recomputed_raw_proof_cannot_be_overruled_by_saved_true(self):
        trial,proof,snapshot=self.independent_context();proof['grasp_success_verified']=False
        with patch('apple_transfer_auto_backend.read',return_value=trial), \
                patch('audit_preclose_identity_lift_follow.audit',return_value=proof):
            with self.assertRaisesRegex(ValueError,'contact_and_multiframe_lift_not_verified'):self.backend.verify_grasp()
    def test_only_source_inference_observer_receives_roi_flag(self):
        b=self.backend;b.config=copy.deepcopy(CONFIG);b.config['policy']['near_grasp_roi_confirmation']=True
        b.observer=None;b.stage='test';b.record=Mock()
        with patch('apple_transfer_auto_backend.subprocess.Popen') as popen:
            for infer in (False,True):
                b.start_observer('observe-'+str(infer),infer)
                self.assertEqual('--confirm-apple-roi' in popen.call_args.args[0],infer)
                self.assertEqual('--near-apple-geometry' in popen.call_args.args[0],infer)
                if infer:
                    args=popen.call_args.args[0]
                    self.assertEqual(args[args.index('--near-apple-gap-m')+1],'0.15')
                b.observer=None;b.observer_log.close();b.observer_log=None


if __name__=='__main__': unittest.main()
