import copy
import unittest
from run_apple_close_lift_trial import lift_allowed,owned_observer_command
from pathlib import Path
from types import SimpleNamespace


class Tests(unittest.TestCase):
    def setUp(self):
        self.result=dict(status='passed',grasp_closure_commanded=True,
            closure_evidence=dict(state='contact_candidate',observed_opening_rad=.41,
                stable_samples=6,stable_duration_s=.7,closing_travel_rad=.24))
    def test_contact_allows_verification_lift(self):self.assertTrue(lift_allowed(self.result))
    def test_empty_close_blocks_lift(self):
        self.result['closure_evidence'].update(state='closed_without_contact_evidence',observed_opening_rad=0)
        self.assertFalse(lift_allowed(self.result))
    def test_unsent_command_blocks_lift(self):
        self.result['grasp_closure_commanded']=False
        self.assertFalse(lift_allowed(self.result))
    def test_unstable_blocks_lift(self):
        self.result['closure_evidence']['stable_duration_s']=.2
        self.assertFalse(lift_allowed(self.result))
    def test_failed_status_blocks_lift(self):
        self.result['status']='failed'
        self.assertFalse(lift_allowed(self.result))
    def test_owned_observer_roi_is_explicit_and_new_task_bound(self):
        args=SimpleNamespace(model_python='/model-python',registry='registry',environment='environment',
                             room='bedroom',tracking_task_id='NEW-task',confirm_apple_roi=True)
        command=owned_observer_command(Path('/code'),Path('/NEW-recording'),'NEW-epoch',args)
        self.assertIn('--confirm-apple-roi',command)
        self.assertEqual(command[command.index('--tracking-epoch')+1],'NEW-epoch')
        self.assertEqual(command[command.index('--identity-tracking-task')+1],'NEW-task')
        self.assertNotIn('--observer-recording',command)
    def test_legacy_owned_observer_has_no_roi_without_opt_in(self):
        args=SimpleNamespace(model_python='/model-python',registry='registry',environment='environment',
                             room='bedroom',tracking_task_id='task',confirm_apple_roi=False)
        self.assertNotIn('--confirm-apple-roi',owned_observer_command(Path('/code'),Path('/recording'),'epoch',args))

    def test_new_owned_observer_geometry_handoff_is_explicit(self):
        args=SimpleNamespace(model_python='/python',registry='registry',environment='environment',
            room='bedroom',tracking_task_id='task',confirm_apple_roi=False,
            near_apple_geometry=True,near_apple_gap_m=.15)
        command=owned_observer_command(Path('/code'),Path('/recording'),'epoch',args)
        self.assertIn('--near-apple-geometry',command)
        self.assertEqual(command[command.index('--near-apple-gap-m')+1],'0.15')


if __name__=='__main__':unittest.main()
