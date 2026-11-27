import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from can_release_timing import publish_once,reviewed_plan


class ReleaseTimingTests(unittest.TestCase):
    def test_expired_images_are_reacquired_before_one_publication(self):
        acquire=Mock(side_effect=[ValueError('fresh_carry_images_required'),ValueError('fresh_current_carry_joint_feedback_required'),{'current':'actual-new-frame'}])
        publish=Mock(return_value='sent');expired=Mock()
        self.assertEqual(publish_once(acquire,publish,expired),'sent')
        self.assertEqual(publish.call_count,1);self.assertEqual(acquire.call_count,3)
        self.assertEqual(expired.call_count,2)
    def test_publication_failure_never_replays_release(self):
        acquire=Mock(return_value={'current':'frame'});publish=Mock(side_effect=RuntimeError('transport_failed'))
        with self.assertRaises(RuntimeError):publish_once(acquire,publish,Mock())
        self.assertEqual(acquire.call_count,1);self.assertEqual(publish.call_count,1)
    def test_invalid_support_is_not_hidden_as_a_time_problem(self):
        acquire=Mock(side_effect=ValueError('actual_can_not_over_safe_table_support'));publish=Mock()
        with self.assertRaises(ValueError):publish_once(acquire,publish,Mock())
        self.assertEqual(acquire.call_count,1);publish.assert_not_called()
    def test_never_fresh_stops_without_a_command(self):
        acquire=Mock(side_effect=ValueError('fresh_carry_images_required'));publish=Mock()
        with self.assertRaisesRegex(ValueError,'no_command_sent'):publish_once(acquire,publish,Mock())
        self.assertEqual(acquire.call_count,5);publish.assert_not_called()
    def test_reviewed_release_cannot_repeat_an_earlier_open_command(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'place';source.mkdir()
            r=dict(status='stopped_requires_current_state_review',context_run_id='run',stage='release_once_from_current_rgbd',
                release_commands_sent=0,hand_released=False,prepare=dict(status='passed'))
            (source/'result.json').write_text(json.dumps(r));(source/'journal.jsonl').write_text('')
            (source/'placement-plan.json').write_text(json.dumps(dict(class_name='canned_juice')))
            self.assertEqual(reviewed_plan(source,'run',root)['class_name'],'canned_juice')
            (source/'journal.jsonl').write_text(json.dumps(dict(event='can_release_sent'))+'\n')
            with self.assertRaises(ValueError):reviewed_plan(source,'run',root)


if __name__=='__main__':unittest.main()
