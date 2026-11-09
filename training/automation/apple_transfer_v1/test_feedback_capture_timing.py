import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from apple_transfer_auto_backend import LiveBackend
from inspect_transfer_ready import validate_sample_times
from verify_source_timing_readonly import assess_grasp_plan


class FeedbackTimesTest(unittest.TestCase):
    def test_exact_old_gate_is_preserved(self):
        self.assertEqual(validate_sample_times(600_000_000,980_000_000,1_000_000_000)['joints'],.4)
        for joint, transform in [(599_000_000,980_000_000), (990_000_000,599_000_000),
                                 (1_021_000_000,980_000_000), (990_000_000,1_021_000_000)]:
            with self.subTest(joint=joint),self.assertRaises(RuntimeError):
                validate_sample_times(joint,transform,1_000_000_000)

    def test_source_clock_offset_cannot_be_masked_by_fresh_image_stamp(self):
        # Joint snapshot in the failed trial was >.5s old at consumption.
        with self.assertRaises(RuntimeError):
            validate_sample_times(490_000_000,990_000_000,1_000_000_000)


class ConsumerTimesTest(unittest.TestCase):
    def test_snapshot_freshness_is_checked_after_observation_disk_read(self):
        with tempfile.TemporaryDirectory() as root:
            b=LiveBackend.__new__(LiveBackend)
            b.recording=Path(root);b.raw_observer=False;b.epoch='epoch';b.task_id='task'
            b.observer=Mock();b.observer.poll.return_value=None
            capture=Path(root)/'000001';capture.mkdir()
            snapshot=dict(streams=dict(rgb=dict(stamp_ns=900_000_000),depth=dict(stamp_ns=900_000_000)),
                joint_stamp_ns=900_000_000,tracking_epoch='epoch',
                tracking_transforms=dict(map_from_base_at_depth=dict(stamp_ns=900_000_000,
                    source_frame='base_link',target_frame='map',translation=[0,0,0],quaternion=[0,0,0,1])))
            events=[]
            def read(path):
                events.append(path.name)
                return copy.deepcopy(snapshot) if path.name=='snapshot.json' else {}
            def gate(*args):
                self.assertEqual(events,['snapshot.json','observation.json'])
                raise ValueError('expired_after_disk_read')
            with patch('apple_transfer_auto_backend.latest_capture',return_value=capture/'observation.json'), \
                    patch('apple_transfer_auto_backend.read',side_effect=read), \
                    patch('apple_transfer_auto_backend.fresh_snapshot',side_effect=gate), \
                    patch('apple_transfer_auto_backend.time.monotonic',side_effect=[0,0,2]), \
                    patch('apple_transfer_auto_backend.time.sleep'):
                with self.assertRaisesRegex(RuntimeError,'expired_after_disk_read'):b.latest(timeout=1)


class ReadonlyGeometryReportingTest(unittest.TestCase):
    def test_geometry_rejection_is_explicit_not_grasp_success(self):
        with patch('verify_source_timing_readonly.plan_grasp',side_effect=ValueError('apple_not_on_requested_source_table')) as gate:
            report=assess_grasp_plan({}, {}, {}, {}, {})
        gate.assert_called_once()
        self.assertFalse(report['grasp_plan_eligible'])
        self.assertEqual(report['grasp_plan_rejection'],'apple_not_on_requested_source_table')
        self.assertNotIn('measured_grasp_palm_height_m',report)

    def test_success_and_unexpected_failures_are_not_fabricated(self):
        with patch('verify_source_timing_readonly.plan_grasp',return_value=dict(palm_height_m=.53)):
            self.assertEqual(assess_grasp_plan({}, {}, {}, {}, {}),dict(grasp_plan_eligible=True,
                grasp_plan_rejection=None,measured_grasp_palm_height_m=.53))
        with patch('verify_source_timing_readonly.plan_grasp',side_effect=RuntimeError('broken_pipeline')):
            with self.assertRaisesRegex(RuntimeError,'broken_pipeline'):
                assess_grasp_plan({}, {}, {}, {}, {})


if __name__=='__main__':unittest.main()
