"""Parent-owned recording integration without motion or a detector dependency."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from PIL import Image

from audit_preclose_identity_lift_follow import audit
from run_apple_close_lift_trial import add_independent_lift_verification
from test_preclose_identity_lift_follow import snapshot,scene


class ParentBindingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();root=Path(self.temp.name)
        self.trial=root/'new-close-lift';self.trial.mkdir()
        self.recording=root/'parent-source-observe';self.recording.mkdir()
        self.before=(dict(tracking_epoch='new-live-trial',identity_tracking_task_id='task',
                          identity_tracking=dict(identity_confirmed=True),detections=[dict(name='apple',confidence=.9)]),
                     snapshot(1_000_000_000,.25,.46),
                     dict(center_m=[0,0,.5],radius_m=.04,stamp_ns=1_000_000_000))
        source=self.recording/'000000';source.mkdir()
        close=dict(status='passed',grasp_closure_commanded=True,
                   closure_evidence=dict(state='contact_candidate',observed_opening_rad=.4,
                       stable_samples=6,stable_duration_s=.7,closing_travel_rad=.25),
                   closure_identity_evidence=dict(evidence=str(source/'observation.json'),stamp_ns=1_000_000_000,age_s=.1))
        self.result=dict(status='visual_verification_inconclusive',grasp_success_verified=False,
                         close=close,lift=dict(status='passed'),observer_owned=False,
                         observation_directory=str(self.recording),tracking_epoch='new-live-trial',tracking_task_id='task',
                         lift_completed_at_ns=8_800_000_000,verification_window_end_ns=10_000_000_000,
                         verification_last_complete_capture=str(self.recording/'000003'))
        self.write_result()
        (source/'snapshot.json').write_text(json.dumps(self.before[1]))
        (source/'observation.json').write_text(json.dumps(self.before[0]))
        rgb,depth=scene()
        for i,stamp in enumerate((9_000_000_000,9_500_000_000,10_000_000_000),1):
            frame=self.recording/f'{i:06d}';frame.mkdir()
            (frame/'snapshot.json').write_text(json.dumps(snapshot(stamp,.35,.56)))
            (frame/'observation.json').write_text(json.dumps(dict(detections=[],tracking_epoch='new-live-trial',identity_tracking_task_id='task')))
            Image.fromarray(rgb).save(frame/'rgb.png');np.save(frame/'depth_m.npy',depth)
    def tearDown(self):self.temp.cleanup()
    def write_result(self):(self.trial/'result.json').write_text(json.dumps(self.result))
    def check(self,**overrides):
        kwargs=dict(expected_recording=self.recording,expected_epoch='new-live-trial',expected_task_id='task');kwargs.update(overrides)
        with patch('audit_preclose_identity_lift_follow.measured',return_value=copy.deepcopy(self.before)):
            return audit(self.trial,**kwargs)
    def test_current_parent_recording_is_explicitly_bound_and_measured(self):
        r=self.check();self.assertTrue(r['grasp_success_verified']);self.assertEqual(r['accepted_after_frames'],3)
        self.assertEqual(r['bound_task_id'],'task');self.assertFalse(r['detector_confidence_invented'])
    def test_unbound_external_recording_still_forbidden(self):
        with self.assertRaisesRegex(ValueError,'owned_recording_required'):audit(self.trial)
    def test_wrong_parent_path_task_epoch_or_ownership_rejected(self):
        for change in (dict(expected_recording=self.recording/'other'),dict(expected_epoch='old'),dict(expected_task_id='old')):
            with self.subTest(change=change),self.assertRaises(ValueError):self.check(**change)
        self.result['observer_owned']=True;self.write_result()
        with self.assertRaises(ValueError):self.check()
    def test_wrong_preclose_identity_task_rejected(self):
        self.before[0]['identity_tracking_task_id']='old'
        with self.assertRaisesRegex(ValueError,'preclose_task_or_epoch_mismatch'):self.check()
    def test_wrong_postlift_task_cannot_authorize_continuation(self):
        path=self.recording/'000002/observation.json';v=json.loads(path.read_text());v['identity_tracking_task_id']='old';path.write_text(json.dumps(v))
        self.assertFalse(self.check()['grasp_success_verified'])
    def test_frozen_time_and_completed_index_exclude_future_frames(self):
        self.result['verification_window_end_ns']=9_200_000_000;self.write_result()
        self.assertFalse(self.check()['grasp_success_verified'])
        self.result['verification_window_end_ns']=10_000_000_000
        self.result['verification_last_complete_capture']=str(self.recording/'000001');self.write_result()
        self.assertFalse(self.check()['grasp_success_verified'])
    def test_new_joint_does_not_make_an_earlier_camera_frame_settled(self):
        self.result['lift_completed_at_ns']=9_250_000_000;self.write_result()
        result=self.check()
        self.assertEqual(result['accepted_after_frames'],2)
        self.assertEqual(result['excluded_transition_captures'],[dict(capture='000001',reason='camera_outside_actual_post_endpoint_window')])
        self.assertFalse(result['grasp_success_verified'])
    def test_bad_settled_frame_is_still_rejected_not_hidden_as_transition(self):
        path=self.recording/'000002/observation.json'
        value=json.loads(path.read_text());value['identity_tracking_task_id']='wrong';path.write_text(json.dumps(value))
        result=self.check()
        self.assertEqual(result['excluded_transition_captures'],[])
        self.assertFalse(result['grasp_success_verified']);self.assertEqual(len(result['rejected']),1)
    def test_two_mm_joint_endpoint_error_does_not_discard_true_lift_evidence(self):
        for path in self.recording.glob('00000[123]/snapshot.json'):
            value=json.loads(path.read_text());value['joints']['arm_lift_joint']=.348
            path.write_text(json.dumps(value))
        result=self.check()
        self.assertTrue(result['grasp_success_verified']);self.assertEqual(result['accepted_after_frames'],3)
    def test_roi_derived_identity_is_not_trusted_without_recomputed_proof(self):
        self.before[0]['identity_tracking']['measurement']=dict(confidence_basis='same_frame_near_grasp_roi')
        with self.assertRaisesRegex(ValueError,'bound_preclose_roi_identity_required'):self.check()
    def test_new_evidence_does_not_rewrite_original_detector_failure(self):
        original=(self.trial/'result.json').read_text()
        canonical=self.trial/'visual-lift-verification.json';canonical.write_text('{"grasp_success_verified": false}')
        with patch('audit_preclose_identity_lift_follow.measured',return_value=copy.deepcopy(self.before)):
            selected=add_independent_lift_verification(self.trial,self.recording,'new-live-trial','task',external_observer=True)
        self.assertTrue(selected['effective_grasp_success_verified'])
        self.assertEqual((self.trial/'result.json').read_text(),original)
        self.assertFalse(json.loads(canonical.read_text())['grasp_success_verified'])
        with self.assertRaisesRegex(ValueError,'already_exists'):
            add_independent_lift_verification(self.trial,self.recording,'new-live-trial','task',external_observer=True)


if __name__=='__main__':unittest.main()
