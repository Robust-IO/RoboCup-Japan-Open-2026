import copy
import unittest
from unittest.mock import Mock
from capture_processing_budget import ProcessingBudget,result_timing,enforce_final_timing


class Tests(unittest.TestCase):
    def setUp(self):
        self.budget=ProcessingBudget();self.now=1_000_000_000
        self.history={k:[970_000_000] for k in ('rgb','depth','hand')}
        self.args=dict(ready=lambda stamps:True,joint_stamp_ns=788_000_000,mode='detector')
        self.snapshot=dict(streams=dict(rgb=dict(stamp_ns=970_000_000),depth=dict(stamp_ns=970_000_000)),
                           joint_stamp_ns=788_000_000)

    def test_bootstrap_does_not_guess200ms_and_starve209ms_joints(self):
        stamps,choice=self.budget.select(self.history,self.now,**self.args)
        self.assertEqual(stamps,(970_000_000,)*3);self.assertTrue(choice['bootstrap'])
        self.assertEqual(choice['applied_processing_reserve_ns'],0)

    def test_measured90ms_hint_allows212ms_joint_age(self):
        self.budget.observe('detector',.09)
        stamps,choice=self.budget.select(self.history,self.now,**self.args)
        self.assertIsNotNone(stamps);self.assertFalse(choice['forecast_fallback_used'])
        self.assertLessEqual(choice['estimated_processing_reserve_ns'],95_000_001)

    def test_pessimistic_forecast_cannot_permanently_veto_valid_start_frame(self):
        self.budget.observe('detector',.25)
        stamps,choice=self.budget.select(self.history,self.now,**self.args)
        self.assertIsNotNone(stamps);self.assertTrue(choice['forecast_fallback_used'])
        # Actual90ms processing succeeds, even though the prior forecast was255ms.
        timing=result_timing(self.snapshot,self.now+90_000_000,.09)
        self.assertTrue(timing['freshness_passed']);self.assertAlmostEqual(timing['joint_age_at_result_s'],.302)

    def test_actual_expiry_is_rejected_even_when_hint_was_optimistic(self):
        self.budget.select(self.history,self.now,**self.args)
        self.assertFalse(result_timing(self.snapshot,self.now+200_000_000,.2)['freshness_passed'])

    def test_fallback_keeps_tf_and400ms_and_future_and_duplicate_constraints(self):
        self.budget.observe('detector',.3)
        for mutation in ('tf','oldjoint','futurejoint','oldimage','sync','duplicate'):
            history=copy.deepcopy(self.history);args=dict(self.args);last=None
            if mutation=='tf':args['ready']=lambda stamps:False
            elif mutation=='oldjoint':args['joint_stamp_ns']=599_999_999
            elif mutation=='futurejoint':args['joint_stamp_ns']=1_030_000_000
            elif mutation=='oldimage':history={k:[599_999_999] for k in history}
            elif mutation=='sync':history['rgb']=[900_000_000]
            else:last=(970_000_000,)*3
            with self.subTest(mutation=mutation):
                self.assertIsNone(self.budget.select(history,self.now,last,**args)[0])

    def test_geometry_and_detector_costs_are_separate(self):
        self.budget.observe('detector',.25);self.budget.observe('geometry',.02)
        self.assertGreater(self.budget.reserve_ns('detector'),self.budget.reserve_ns('geometry'))

    def test_window_forgets_old_slow_sample_and_large_cost_is_only_a_hint(self):
        self.budget.observe('detector',2.)
        self.assertLess(self.budget.reserve_ns('detector'),400_000_000)
        self.assertIsNotNone(self.budget.select(self.history,self.now,**self.args)[0])
        for _ in range(16):self.budget.observe('detector',.02)
        self.assertLess(self.budget.reserve_ns('detector'),30_000_000)

    def test_result_checks_original_images_and_original_joint_without_stamp_changes(self):
        original=copy.deepcopy(self.snapshot)
        for change in ('rgb','depth','joint','futurejoint','sync','elapsed'):
            s=copy.deepcopy(self.snapshot);elapsed=.1
            if change in ('rgb','depth'):s['streams'][change]['stamp_ns']=699_999_999
            elif change=='joint':s['joint_stamp_ns']=699_999_999
            elif change=='futurejoint':s['joint_stamp_ns']=1_130_000_000
            elif change=='sync':s['streams']['rgb']['stamp_ns']=930_000_000
            else:elapsed=.401
            with self.subTest(change=change):self.assertFalse(result_timing(s,1_100_000_000,elapsed)['freshness_passed'])
        self.assertEqual(self.snapshot,original)

    def test_processing_elapsed_still_fails_if_ros_clock_stops(self):
        self.assertFalse(result_timing(self.snapshot,self.now,.401)['freshness_passed'])

    def test_final_expiry_clears_identity_and_geometric_anchor_not_raw_scores(self):
        observer=Mock();observer.geometry=Mock()
        result=dict(timing={},eligible_fresh_candidate=True,detections=[dict(name='apple',confidence=.95)],
            identity_tracking=dict(identity_confirmed=True),center_evidence=dict(coarse_center_consistent=True))
        enforce_final_timing(result,result_timing(self.snapshot,self.now+200_000_000,.2),observer)
        self.assertFalse(result['eligible_fresh_candidate']);self.assertFalse(result['identity_tracking']['identity_confirmed'])
        self.assertEqual(result['detections'][0]['confidence'],.95)
        observer.track.reset.assert_called_once();observer.geometry.reset.assert_called_once()

    def test_success_does_not_reset_tracker_or_authorize_motion(self):
        observer=Mock();result=dict(timing={},identity_tracking=dict(identity_confirmed=True))
        enforce_final_timing(result,result_timing(self.snapshot,self.now+50_000_000,.05),observer)
        self.assertTrue(result['identity_tracking']['identity_confirmed']);self.assertFalse(result['timing']['motion_authorized'])
        observer.track.reset.assert_not_called()

    def test_invalid_costs_do_not_relax_freshness(self):
        for value in (-.1,True,float('nan'),float('inf'),'0.1'):
            with self.subTest(value=value),self.assertRaises(ValueError):self.budget.observe('detector',value)

    def test_live_scheduler_waits_briefly_for_new_feedback_instead_of_repeated_expired_work(self):
        self.budget.observe('detector',.1)
        args=dict(self.args,joint_stamp_ns=700_000_000)
        self.assertIsNone(self.budget.select(self.history,self.now,monotonic_s=1.,**args)[0])
        stamps,choice=self.budget.select(self.history,self.now+50_000_000,monotonic_s=1.05,
            **dict(args,joint_stamp_ns=840_000_000))
        self.assertIsNotNone(stamps);self.assertFalse(choice['forecast_fallback_used'])
        self.assertEqual(choice['publication_consumer_scheduling_headroom_ns'],30_000_000)

    def test_feedback_preference_never_starves_on_pessimistic_cost_forecast(self):
        self.budget.observe('detector',.3)
        self.assertIsNone(self.budget.select(self.history,self.now,monotonic_s=2.,**self.args)[0])
        stamps,choice=self.budget.select(self.history,self.now,monotonic_s=2.26,**self.args)
        self.assertIsNotNone(stamps);self.assertTrue(choice['forecast_fallback_used'])
        self.assertFalse(choice['bounded_new_feedback_wait'])

    def test_bounded_wait_fallback_still_rejects_original_expired_joint(self):
        self.budget.observe('detector',.2);args=dict(self.args,joint_stamp_ns=500_000_000)
        self.budget.select(self.history,self.now,monotonic_s=3.,**args)
        self.assertIsNone(self.budget.select(self.history,self.now,monotonic_s=3.3,**args)[0])

    def test_bootstrap_does_not_add_guessed_reserve_or_wait(self):
        stamps,choice=self.budget.select(self.history,self.now,monotonic_s=4.,**self.args)
        self.assertIsNotNone(stamps);self.assertFalse(choice['bounded_new_feedback_wait'])
        self.assertEqual(choice['publication_consumer_scheduling_headroom_ns'],0)

    def test_invalid_monotonic_clock_rejected(self):
        for value in (float('nan'),float('inf'),True,-1):
            with self.assertRaises(ValueError):self.budget.select(self.history,self.now,monotonic_s=value,**self.args)


if __name__=='__main__':unittest.main()
